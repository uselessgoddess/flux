#![feature(let_chains)]

use flux::prelude::*;

use rapier::{
  dynamics::{
    ImpulseJointSet, MultibodyJointSet, RigidBodyBuilder, RigidBodySet,
  },
  geometry::{ColliderBuilder, ColliderSet},
  prelude::*,
};

use harness::Harness;

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

fn stand() -> Stand {
  let mut bodies = RigidBodySet::new();
  let mut colliders = ColliderSet::new();
  let impulse_joints = ImpulseJointSet::new();
  let multibody_joints = MultibodyJointSet::new();

  let num = 10;
  let rad = 0.2;

  let subdiv = 1.0 / (num as f32);

  for i in 0usize..num {
    let (x, y) = (i as f32 * subdiv * std::f32::consts::PI * 2.0).sin_cos();

    let rb = RigidBodyBuilder::dynamic()
      .translation(vector![x, y, 0.0])
      .linvel(vector![x * 10.0, y * 10.0, 0.0])
      .angvel(Vector::z() * 100.0)
      .linear_damping((i + 1) as f32 * subdiv * 10.0)
      .angular_damping((num - i) as f32 * subdiv * 10.0);
    let rb_handle = bodies.insert(rb);

    let co = ColliderBuilder::cuboid(rad, rad, rad);
    colliders.insert_with_parent(co, rb_handle, &mut bodies);
  }

  let mut harness =
    Harness::new(bodies, colliders, impulse_joints, multibody_joints);
  harness.physics.gravity = Vector::zeros();

  Stand::new(harness).add_plugin(fluid_pipeline())
}

use nalgebra::Vector3;

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

fn fluid_pipeline() -> FluidsPlugin {
  use salva::integrations::rapier::FluidsPipeline;

  const PARTICLE_RADIUS: f32 = 0.025;
  const SMOOTHING_FACTOR: f32 = 2.0;

  let mut fluids_pipeline =
    FluidsPipeline::new(PARTICLE_RADIUS, SMOOTHING_FACTOR);

  let nparticles = 30;
  let custom_force1 = CustomForceField { origin: Point3::new(1.0, 0.0, 0.0) };
  let custom_force2 = CustomForceField { origin: Point3::new(-1.0, 0.0, 0.0) };
  let mut fluid =
    cube_fluid(nparticles, nparticles, nparticles, PARTICLE_RADIUS, 1000.0);
  fluid.nonpressure_forces.push(Box::new(custom_force1));
  fluid.nonpressure_forces.push(Box::new(custom_force2));
  let _fluid_handle = fluids_pipeline.liquid_world.add_fluid(fluid);

  let mut plugin = FluidsPlugin::new();
  plugin.set_pipeline(fluids_pipeline);
  plugin
}

use nalgebra::{Point3, Unit};

struct CustomForceField {
  origin: Point3<f32>,
}

use salva::{
  object::{Boundary, Fluid},
  solver::NonPressureForce,
};

impl NonPressureForce for CustomForceField {
  fn solve(
    &mut self,
    _timestep: &salva::TimestepManager,
    _kernel_radius: f32,
    _fluid_fluid_contacts: &salva::geometry::ParticlesContacts,
    _fluid_boundaries_contacts: &salva::geometry::ParticlesContacts,
    fluid: &mut Fluid,
    _boundaries: &[Boundary],
    _densities: &[f32],
  ) {
    for (pos, acc) in fluid.positions.iter().zip(fluid.accelerations.iter_mut())
    {
      if let Some((dir, dist)) = Unit::try_new_and_get(self.origin - pos, 0.1) {
        *acc += *dir / dist;
      }
    }
  }

  fn apply_permutation(&mut self, _permutation: &[usize]) {}
}
