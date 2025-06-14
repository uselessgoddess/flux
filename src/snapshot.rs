use {
  crate::prelude::*,
  harness::{Harness, PhysicsState},
  rapier::{
    dynamics::{
      ImpulseJointSet, IslandManager, MultibodyJointSet, RigidBodySet,
    },
    geometry::{ColliderSet, DefaultBroadPhase, NarrowPhase},
  },
};

pub trait Snapshot {
  fn draw(&self, graphics: &mut Gizmos);
}

pub struct PhysicsSnapshot {
  pub timestep_id: usize,
  pub broad_phase: DefaultBroadPhase,
  pub narrow_phase: NarrowPhase,
  pub island_manager: IslandManager,
  pub bodies: RigidBodySet,
  pub colliders: ColliderSet,
  pub impulse_joints: ImpulseJointSet,
  pub multibody_joints: MultibodyJointSet,
}

impl PhysicsSnapshot {
  pub fn capture(harness: &Harness) -> Self {
    let PhysicsState {
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      ..
    } = &harness.physics;
    Self {
      timestep_id: harness.state.timestep_id,
      island_manager: islands.clone(),
      broad_phase: broad_phase.clone(),
      narrow_phase: narrow_phase.clone(),
      bodies: bodies.clone(),
      colliders: colliders.clone(),
      impulse_joints: impulse_joints.clone(),
      multibody_joints: multibody_joints.clone(),
    }
  }
}

impl Snapshot for PhysicsSnapshot {
  fn draw(&self, graphics: &mut Gizmos) {
    for (_, body) in self.bodies.iter() {
      for &handle in body.colliders() {
        if let Some(collider) = self.colliders.get(handle)
          && let Some(cuboid) = collider.shape().as_cuboid()
        {
          let pos = body.translation();
          let rot = body.rotation();
          graphics.cuboid(
            Transform {
              translation: Vec3::from_slice(pos.as_slice()),
              rotation: Quat::from_slice(rot.coords.as_slice()),
              scale: Vec3::from_slice(cuboid.half_extents.as_slice()),
            },
            Color::srgb(1.0, 0.0, 0.0),
          );
        }
      }
    }
  }
}
