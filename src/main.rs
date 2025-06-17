#![feature(let_chains)]

use flux::prelude::*;

use rapier::{
  dynamics::{
    ImpulseJointSet, MultibodyJointSet, RigidBodyBuilder, RigidBodySet,
  },
  geometry::{ColliderBuilder, ColliderSet},
  prelude::*,
};

use {
  flux::harness::Fluids,
  harness::Harness,
  nalgebra::Isometry3,
  salva::{
    integrations::rapier::{ColliderSampling, FluidsPipeline},
    object::{Boundary, interaction_groups::InteractionGroups},
    sampling,
  },
};

fn main() {
  let mut app = flux::app();
  app.add_systems(Startup, setup);

  let (harness, fluids) = stand();

  stand::plugin(&mut app, harness, fluids);
  app.run();
}

fn setup(mut commands: Commands) {
  commands.spawn((
    Camera3d::default(),
    PanOrbitCamera::default(),
    Transform::IDENTITY
      .looking_at(Vec3::new(2.0, 2.5, 20.0), Vec3::new(2.0, 2.5, 0.0)),
  ));
}

const PARTICLE_RADIUS: f32 = 0.05;
const SMOOTHING_FACTOR: f32 = 2.0;

fn stand() -> (Harness, Fluids) {
  let mut bodies = RigidBodySet::new();
  let mut colliders = ColliderSet::new();
  let impulse_joints = ImpulseJointSet::new();
  let multibody_joints = MultibodyJointSet::new();
  let mut fluids_pipeline =
    FluidsPipeline::new(PARTICLE_RADIUS, SMOOTHING_FACTOR);

  let ground_thickness = 0.2;
  let ground_half_width = 2.5;

  let ground_shape =
    SharedShape::cuboid(ground_half_width, ground_thickness, ground_half_width);

  let ground_body = RigidBodyBuilder::fixed().build();
  let ground_handle = bodies.insert(ground_body);

  let samples =
    sampling::shape_volume_ray_sample(&*ground_shape, PARTICLE_RADIUS).unwrap();
  let co = ColliderBuilder::new(ground_shape)
    .position(Isometry3::translation(0.0, -10.0, 0.0))
    .build();
  let co_handle = colliders.insert_with_parent(co, ground_handle, &mut bodies);
  let bo_handle = fluids_pipeline
    .liquid_world
    .add_boundary(Boundary::new(Vec::new(), InteractionGroups::default()));

  fluids_pipeline.coupling.register_coupling(
    bo_handle,
    co_handle,
    ColliderSampling::StaticSampling(samples),
  );

  let mut harness =
    Harness::new(bodies, colliders, impulse_joints, multibody_joints);
  harness.integration_parameters_mut().dt = 1.0 / 200.0;

  (harness, Fluids::from_pipeline(fluids_pipeline))
}
