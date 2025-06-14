#![feature(let_chains)]

use {flux::prelude::*, std::num::NonZero};

use rapier::{
  dynamics::{
    ImpulseJointSet, MultibodyJointSet, RigidBodyBuilder, RigidBodySet,
  },
  geometry::{ColliderBuilder, ColliderSet},
  prelude::*,
};

use salva::{
  integrations::rapier::{ColliderSampling, FluidsPipeline},
  object::{Boundary, Fluid, interaction_groups::InteractionGroups},
  sampling,
  solver::ArtificialViscosity,
};

use {
  flux::harness::Fluids,
  harness::Harness,
  nalgebra::{Isometry3, Point3, Vector3},
};

fn main() {
  let mut app = flux::app();
  app.add_systems(Startup, setup);

  let rx = stand().detach_thread();

  Stand::plugin(&mut app, rx);
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

fn stand() -> Stand {
  let mut bodies = RigidBodySet::new();
  let mut colliders = ColliderSet::new();
  let impulse_joints = ImpulseJointSet::new();
  let multibody_joints = MultibodyJointSet::new();
  let mut fluids_pipeline =
    FluidsPipeline::new(PARTICLE_RADIUS, SMOOTHING_FACTOR);

  let ground_thickness = 0.2;
  let ground_half_width = 2.5;

  use salva::solver::{Becker2009Elasticity, XSPHViscosity};

  let elasticity: Becker2009Elasticity =
    Becker2009Elasticity::new(500_000.0, 0.3, true);
  let viscosity = XSPHViscosity::new(0.5, 1.0);

  let nparticles = 30;
  let mut fluid =
    cube_fluid(nparticles, nparticles, nparticles, PARTICLE_RADIUS, 1000.0);
  fluid.nonpressure_forces.push(Box::new(elasticity));
  fluid.nonpressure_forces.push(Box::new(viscosity));
  fluid.transform_by(&Isometry3::translation(
    0.0,
    ground_thickness + nparticles as f32 * PARTICLE_RADIUS,
    0.0,
  ));
  let viscosity = ArtificialViscosity::new(1.0, 0.0);
  fluid.nonpressure_forces.push(Box::new(viscosity));
  let _fluid_handle = fluids_pipeline.liquid_world.add_fluid(fluid);

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

  Stand::new(harness, Fluids::from_pipeline(fluids_pipeline))
}

pub fn cube_fluid(
  ni: usize,
  nj: usize,
  nk: usize,
  particle_rad: f32,
  density: f32,
) -> Fluid {
  let mut points = Vec::new();
  let half_extents =
    Vector3::new(ni as f32, nj as f32, nk as f32) * particle_rad;

  for i in 0..ni {
    for j in 0..nj {
      for k in 0..nk {
        let x = (i as f32) * particle_rad * 2.0;
        let y = (j as f32) * particle_rad * 2.0;
        let z = (k as f32) * particle_rad * 2.0;
        points.push(
          Point3::new(x, y, z) + Vector3::repeat(particle_rad) - half_extents,
        );
      }
    }
  }

  use salva::object::interaction_groups::InteractionGroups;

  Fluid::new(points, particle_rad, density, InteractionGroups::default())
}
