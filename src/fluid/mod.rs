use {
  crate::prelude::*,
  harness::{PhysicsEvents, PhysicsState, RunState, SharedSnapshot},
  rapier::math::{Point, Real},
  salva::{
    LiquidWorld, integrations::rapier::FluidsPipeline, object::FluidHandle,
  },
};

/// A user-defined callback executed at each frame.
pub type FluidCallback = Box<
  dyn FnMut(&mut PhysicsState, &PhysicsEvents, &mut FluidsPipeline, &RunState)
    + Send,
>;

impl Default for FluidsPlugin {
  fn default() -> Self {
    Self::new()
  }
}

/// A plugin for rendering fluids with the Rapier harness.
pub struct FluidsPlugin {
  pipeline: FluidsPipeline,
  callbacks: Vec<FluidCallback>,
  step_time: f64,
}

impl FluidsPlugin {
  /// Initializes the plugin.
  pub fn new() -> Self {
    Self {
      pipeline: FluidsPipeline::new(0.025, 2.0),
      callbacks: Vec::new(),
      step_time: 0.0,
    }
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
}

pub struct Snapshot {
  pub fluids: Vec<(FluidHandle, Fluid)>,
  pub particle_radius: f32,
}

impl harness::Snapshot for Snapshot {}

impl harness::Plugin for FluidsPlugin {
  fn snapshot(&self) -> SharedSnapshot {
    use salva::object;

    let fluid = |(handle, fluid): (FluidHandle, &object::Fluid)| {
      (handle, Fluid { positions: fluid.positions.to_vec() })
    };

    let world = self.liquid_world();
    SharedSnapshot::new(Snapshot {
      fluids: world.fluids().iter().map(fluid).collect(),
      particle_radius: world.particle_radius(),
    })
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

impl stand::Plugin for FluidsPlugin {}
