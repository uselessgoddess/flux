use {
  crate::{harness::Fluids, prelude::*, stand::flow::ShapeFlow},
  parry::shape::Ball,
  salva::{
    math::{Isometry, Vector},
    object::FluidHandle,
    parry, solver,
  },
};

pub fn setup(
  mut fluids: NonSendMut<Fluids>,
  mut commands: Commands,
  mut run: Local<bool>,
) {
  if *run {
    return;
  } else {
    *run = true;
  }

  let world = &mut fluids.pipeline.liquid_world;
  let particle_radius = world.particle_radius();

  use solver::{Akinci2013SurfaceTension, ArtificialViscosity, XSPHViscosity};

  let viscosity = XSPHViscosity::new(0.5, 0.5);
  let mut fluid = helper::cube_fluid(15, 15, 15, particle_radius, 1000.0);
  fluid.nonpressure_forces.push(Box::new(viscosity));
  fluid.transform_by(&Isometry::translation(0.0, -5.0, 0.0));
  fluid.nonpressure_forces.push(Box::new(ArtificialViscosity::new(1.0, 0.0)));

  let _fluid_handle = world.add_fluid(fluid);

  let surface_tension = Akinci2013SurfaceTension::new(0.1, 1.0);
  let mut fluid = helper::cube_fluid(0, 0, 0, particle_radius, 1000.0);
  fluid.transform_by(&Isometry::translation(0.0, 0.08, 0.0));
  fluid.nonpressure_forces.push(Box::new(surface_tension));
  let handle = world.add_fluid(fluid);

  let flow = ShapeFlow::new(
    Vector::new(-10.0, 0.0, 0.0),
    &Ball::new(0.2),
    particle_radius,
  )
  .unwrap()
  .with_velocity(Vector::new(1.0, 1.0, 0.0) * 5.0);
  commands.spawn(Inflow { flow, handle });
}

#[derive(Component)]
pub struct Inflow {
  flow: ShapeFlow,
  handle: FluidHandle,
}

pub fn update(flows: Query<&Inflow>, mut fluids: NonSendMut<Fluids>) {
  let world = &mut fluids.pipeline.liquid_world;

  for &Inflow { ref flow, handle } in flows.iter() {
    flow.emit(world, handle);
  }
}
