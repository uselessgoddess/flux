use {
  crossbeam::channel::Receiver,
  rapier::{
    dynamics::{
      CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager,
      MultibodyJointSet, RigidBodySet,
    },
    geometry::{
      ColliderSet, CollisionEvent, ContactForceEvent, DefaultBroadPhase,
      NarrowPhase,
    },
    math::{Real, Vector},
    pipeline::{PhysicsHooks, PhysicsPipeline, QueryPipeline},
  },
};

pub struct PhysicsState {
  pub islands: IslandManager,
  pub broad_phase: DefaultBroadPhase,
  pub narrow_phase: NarrowPhase,
  pub bodies: RigidBodySet,
  pub colliders: ColliderSet,
  pub impulse_joints: ImpulseJointSet,
  pub multibody_joints: MultibodyJointSet,
  pub ccd_solver: CCDSolver,
  pub pipeline: PhysicsPipeline,
  pub query_pipeline: QueryPipeline,
  pub integration_parameters: IntegrationParameters,
  pub gravity: Vector<Real>,
  pub hooks: Box<dyn PhysicsHooks>,
}

impl Default for PhysicsState {
  fn default() -> Self {
    Self::new()
  }
}

impl PhysicsState {
  pub fn new() -> Self {
    Self {
      islands: IslandManager::new(),
      broad_phase: DefaultBroadPhase::new(),
      narrow_phase: NarrowPhase::new(),
      bodies: RigidBodySet::new(),
      colliders: ColliderSet::new(),
      impulse_joints: ImpulseJointSet::new(),
      multibody_joints: MultibodyJointSet::new(),
      ccd_solver: CCDSolver::new(),
      pipeline: PhysicsPipeline::new(),
      query_pipeline: QueryPipeline::new(),
      integration_parameters: IntegrationParameters::default(),
      gravity: Vector::y() * -9.81,
      hooks: Box::new(()),
    }
  }
}

pub struct PhysicsEvents {
  pub collision_events: Receiver<CollisionEvent>,
  pub contact_force_events: Receiver<ContactForceEvent>,
}

impl PhysicsEvents {
  pub fn poll_all(&self) {
    while self.collision_events.try_recv().is_ok() {}
    while self.contact_force_events.try_recv().is_ok() {}
  }
}
