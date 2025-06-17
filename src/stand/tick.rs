use {
  super::{Sim, Wait},
  crate::{harness::Fluids, prelude::*},
  salva::{
    math::Isometry,
    solver::{Akinci2013SurfaceTension, ArtificialViscosity},
  },
};

pub fn setup(mut fluids: NonSendMut<Fluids>, mut run: Local<bool>) {
  if *run {
    return;
  } else {
    *run = true;
  }

  let world = &mut fluids.pipeline.liquid_world;
  let particle_radius = world.particle_radius();

  let surface_tension = Akinci2013SurfaceTension::new(1.0, 0.0);
  let viscosity = ArtificialViscosity::new(1.5, 1.0);
  let mut fluid = helper::cube_fluid(7, 7, 7, particle_radius, 1000.0);
  fluid.transform_by(&Isometry::translation(0.0, 0.08, 0.0));
  fluid.nonpressure_forces.push(Box::new(surface_tension));
  fluid.nonpressure_forces.push(Box::new(viscosity));
  let _fluid_handle = world.add_fluid(fluid);
}

pub fn update(
  mut fluids: NonSendMut<Fluids>,
  mut timer: ResMut<Wait>,
  time: Res<Time<Sim>>,
) {
  let world = &mut fluids.pipeline.liquid_world;
  let particle_radius = world.particle_radius();

  if timer.tick(time.delta()).just_finished() {
    use salva::solver::{Becker2009Elasticity, XSPHViscosity};

    let elasticity: Becker2009Elasticity =
      Becker2009Elasticity::new(500_000.0, 0.3, true);
    let viscosity = XSPHViscosity::new(0.5, 1.0);

    let amount = 30;

    let mut fluid =
      helper::cube_fluid(amount, amount, amount, particle_radius, 1000.0);
    fluid.nonpressure_forces.push(Box::new(elasticity));
    fluid.nonpressure_forces.push(Box::new(viscosity));
    fluid.transform_by(&Isometry::translation(
      0.0,
      amount as f32 * particle_radius,
      0.0,
    ));
    fluid.nonpressure_forces.push(Box::new(ArtificialViscosity::new(1.0, 0.0)));

    let _fluid_handle = fluids.pipeline.liquid_world.add_fluid(fluid);
  }
}
