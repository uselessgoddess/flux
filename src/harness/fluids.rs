use {
  crate::{
    harness::{self, PhysicsEvents, PhysicsState, RunState},
    prelude::*,
  },
  rapier::math::{Point, Vector},
  salva::{
    LiquidWorld,
    integrations::rapier::FluidsPipeline,
    object::{BoundaryHandle, FluidHandle},
  },
};

/// A user-defined callback executed at each frame.
pub type FluidCallback = Box<
  dyn FnMut(&mut PhysicsState, &PhysicsEvents, &mut FluidsPipeline, &RunState)
    + Send,
>;

impl Default for Fluids {
  fn default() -> Self {
    Self::new()
  }
}

/// A plugin for rendering fluids with the Rapier harness.
pub struct Fluids {
  pipeline: FluidsPipeline,
  callbacks: Vec<FluidCallback>,
  step_time: f64,
}

impl Fluids {
  /// Initializes the plugin.
  pub fn new() -> Self {
    Self {
      pipeline: FluidsPipeline::new(0.025, 2.0),
      callbacks: Vec::new(),
      step_time: 0.0,
    }
  }

  pub fn from_pipeline(pipeline: FluidsPipeline) -> Self {
    Self { pipeline, callbacks: Vec::new(), step_time: 0.0 }
  }

  /// Adds a callback to be executed at each frame.
  pub fn add_callback(
    &mut self,
    f: impl FnMut(&mut PhysicsState, &PhysicsEvents, &mut FluidsPipeline, &RunState)
    + Send
    + 'static,
  ) {
    self.callbacks.push(Box::new(f))
  }

  /// Sets the fluids pipeline used by the harness.
  pub fn set_pipeline(&mut self, pipeline: FluidsPipeline) {
    self.pipeline = pipeline;
    self.pipeline.liquid_world.counters.enable();
  }

  fn liquid_world(&self) -> &LiquidWorld {
    &self.pipeline.liquid_world
  }
}

pub struct Fluid {
  pub positions: Vec<Point<Real>>,
  pub velocities: Vec<Vector<Real>>,
}

pub struct Boundary {
  pub positions: Vec<Point<Real>>,
}

pub struct FluidsSnapshot {
  pub fluids: Vec<(FluidHandle, Fluid)>,
  pub boundaries: Vec<(BoundaryHandle, Boundary)>,
  pub particle_radius: f32,
}

impl harness::Plugin for Fluids {
  type Snapshot = FluidsSnapshot;

  fn snapshot(&self) -> Self::Snapshot {
    use salva::object;

    let fluid = |(handle, fluid): (FluidHandle, &object::Fluid)| {
      (
        handle,
        Fluid {
          positions: fluid.positions.to_vec(),
          velocities: fluid.velocities.to_vec(),
        },
      )
    };
    let boundary = |(handle, boundary): (BoundaryHandle, &object::Boundary)| {
      (handle, Boundary { positions: boundary.positions.to_vec() })
    };

    let world = self.liquid_world();
    FluidsSnapshot {
      fluids: world.fluids().iter().map(fluid).collect(),
      boundaries: world.boundaries().iter().map(boundary).collect(),
      particle_radius: world.particle_radius(),
    }
  }

  fn run_callbacks(
    &mut self,
    physics: &mut PhysicsState,
    physics_events: &PhysicsEvents,
    run_state: &RunState,
  ) {
    for callback in &mut self.callbacks {
      callback(physics, physics_events, &mut self.pipeline, run_state)
    }
  }

  fn step(&mut self, physics: &mut PhysicsState, _run_state: &RunState) {
    let step_time = instant::now();
    self.pipeline.step(
      &physics.gravity,
      physics.integration_parameters.dt,
      &physics.colliders,
      &mut physics.bodies,
    );
    self.step_time = instant::now() - step_time;
  }

  fn profiling_string(&self) -> String {
    format!("Fluids: {:.2}ms", self.step_time)
  }
}

impl snapshot::Snapshot for FluidsSnapshot {
  fn draw(&self, graphics: &mut Gizmos) {
    let FluidsSnapshot { fluids, boundaries, particle_radius } = self;

    // нужно красить по скорости от максимальной до минимальной
    for (_, fluid) in fluids.iter() {
      let velocities: Vec<_> =
        fluid.velocities.iter().map(Vector::magnitude).collect();
      let (min, max) = (
        velocities.iter().copied().min_by(Real::total_cmp).unwrap(),
        velocities.iter().copied().max_by(Real::total_cmp).unwrap(),
      );

      use bevy::math::VectorSpace;

      for (i, particle) in fluid.positions.iter().enumerate() {
        let vel = velocities[i];
        graphics
          .sphere(
            Isometry3d::from_translation(Vec3::from_slice(
              particle.coords.as_slice(),
            )),
            *particle_radius,
            Srgba::rgb(0.0, 0.2, 0.65)
              .lerp(Srgba::rgb(1.0, 0.5, 0.85), vel / max),
          )
          .resolution(4);
      }
    }

    for (_, boundary) in boundaries {
      for particle in boundary
        .positions
        .iter()
        .enumerate()
        .filter_map(flate::<_, { 3 * 3 * 3 }>)
      {
        graphics
          .sphere(
            Isometry3d::from_translation(Vec3::from_slice(
              particle.coords.as_slice(),
            )),
            *particle_radius,
            Color::srgb(1.0, 0.2, 0.65),
          )
          .resolution(4);
      }
    }
  }
}

fn flate<T, const N: usize>((i, t): (usize, T)) -> Option<T> {
  (i % N == 0).then_some(t)
}
