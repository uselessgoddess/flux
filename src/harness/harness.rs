use rapier::{
  dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager,
    MultibodyJointSet, RigidBodySet,
  },
  geometry::{ColliderSet, DefaultBroadPhase, NarrowPhase},
  math::{Real, Vector},
  pipeline::{
    ChannelEventCollector, PhysicsHooks, PhysicsPipeline, QueryPipeline,
  },
};

use super::{PhysicsEvents, PhysicsState};

pub struct RunState {
  #[cfg(feature = "parallel")]
  pub thread_pool: rapier::rayon::ThreadPool,
  #[cfg(feature = "parallel")]
  num_threads: usize,
  pub timestep_id: usize,
  pub time: f32,
}

impl Default for RunState {
  fn default() -> Self {
    Self::new()
  }
}

impl RunState {
  #[cfg(feature = "parallel")]
  pub fn new() -> Self {
    let num_threads = num_cpus::get_physical();

    let thread_pool = rapier::rayon::ThreadPoolBuilder::new()
      .num_threads(num_threads)
      .build()
      .unwrap();

    Self { thread_pool, num_threads, timestep_id: 0, time: 0.0 }
  }

  #[cfg(not(feature = "parallel"))]
  pub fn new() -> Self {
    Self { timestep_id: 0, time: 0.0 }
  }

  #[cfg(feature = "parallel")]
  pub fn num_threads(&self) -> usize {
    self.num_threads
  }

  #[cfg(not(feature = "parallel"))]
  pub fn num_threads(&self) -> usize {
    1
  }

  #[cfg(feature = "parallel")]
  pub fn set_num_threads(&mut self, num_threads: usize) {
    if self.num_threads != num_threads {
      self.thread_pool = rapier::rayon::ThreadPoolBuilder::new()
        .num_threads(num_threads)
        .build()
        .unwrap();
      self.num_threads = num_threads;
    }
  }
}

pub struct Harness {
  pub state: RunState,
  pub physics: PhysicsState,
  max_steps: usize,
  callbacks: Callbacks,
  pub events: PhysicsEvents,
  event_handler: ChannelEventCollector,
}

pub trait Plugin {
  type Snapshot;

  fn snapshot(&self) -> Self::Snapshot;

  fn run_callbacks(
    &mut self,
    physics: &mut PhysicsState,
    events: &PhysicsEvents,
    harness_state: &RunState,
  );

  fn step(&mut self, physics: &mut PhysicsState, run_state: &RunState);

  fn profiling_string(&self) -> String;
}

type Callbacks =
  Vec<Box<dyn FnMut(&mut PhysicsState, &PhysicsEvents, &RunState) + Send>>;

#[allow(dead_code)]
impl Harness {
  pub fn new_empty() -> Self {
    use crossbeam::channel;

    let (collisions, contacts) = (channel::unbounded(), channel::unbounded());
    let physics = PhysicsState::new();
    let state = RunState::new();

    Self {
      state,
      physics,
      max_steps: 1000,
      events: PhysicsEvents {
        collision_events: collisions.1,
        contact_force_events: contacts.1,
      },
      event_handler: ChannelEventCollector::new(collisions.0, contacts.0),
      callbacks: vec![],
    }
  }

  pub fn new(
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
  ) -> Self {
    let mut res = Self::new_empty();
    res.set_world(bodies, colliders, impulse_joints, multibody_joints);
    res
  }

  pub fn set_max_steps(&mut self, max_steps: usize) {
    self.max_steps = max_steps
  }

  pub fn integration_parameters_mut(&mut self) -> &mut IntegrationParameters {
    &mut self.physics.integration_parameters
  }

  pub fn delta(&self) -> f32 {
    self.physics.integration_parameters.dt
  }

  pub fn clear_callbacks(&mut self) {
    self.callbacks.clear();
  }

  pub fn physics_state_mut(&mut self) -> &mut PhysicsState {
    &mut self.physics
  }

  pub fn set_world(
    &mut self,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
  ) {
    self.set_world_with_params(
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      Vector::y() * -9.81,
      (),
    )
  }

  pub fn set_world_with_params(
    &mut self,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    gravity: Vector<Real>,
    hooks: impl PhysicsHooks + 'static,
  ) {
    self.physics.gravity = gravity;
    self.physics.bodies = bodies;
    self.physics.colliders = colliders;
    self.physics.impulse_joints = impulse_joints;
    self.physics.multibody_joints = multibody_joints;
    self.physics.hooks = Box::new(hooks);

    self.physics.islands = IslandManager::new();
    self.physics.broad_phase = DefaultBroadPhase::new();
    self.physics.narrow_phase = NarrowPhase::new();
    self.state.timestep_id = 0;
    self.state.time = 0.0;
    self.physics.ccd_solver = CCDSolver::new();
    self.physics.query_pipeline = QueryPipeline::new();
    self.physics.pipeline = PhysicsPipeline::new();
    self.physics.pipeline.counters.enable();
  }

  pub fn add_callback<
    F: FnMut(&mut PhysicsState, &PhysicsEvents, &RunState) + Send + 'static,
  >(
    &mut self,
    callback: F,
  ) {
    self.callbacks.push(Box::new(callback));
  }

  // #[profiling::function]
  pub fn step(&mut self) {
    let Self { event_handler, physics, .. } = self;
    let mut step = || {
      physics.pipeline.step(
        &physics.gravity,
        &physics.integration_parameters,
        &mut physics.islands,
        &mut physics.broad_phase,
        &mut physics.narrow_phase,
        &mut physics.bodies,
        &mut physics.colliders,
        &mut physics.impulse_joints,
        &mut physics.multibody_joints,
        &mut physics.ccd_solver,
        Some(&mut physics.query_pipeline),
        &*physics.hooks,
        event_handler,
      )
    };

    #[cfg(feature = "parallel")]
    self.state.thread_pool.install(step);

    #[cfg(not(feature = "parallel"))]
    step();

    for callback in &mut self.callbacks {
      callback(&mut self.physics, &self.events, &self.state);
    }

    self.events.poll_all();

    self.state.time += self.physics.integration_parameters.dt;
    self.state.timestep_id += 1;
  }

  pub fn run(&mut self) {
    for _ in 0..self.max_steps {
      self.step();
    }
  }
}
