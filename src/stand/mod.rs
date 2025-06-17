mod tick;

use {
  crate::{
    harness::{Fluids, FluidsSnapshot, Plugin},
    prelude::*,
    snapshot::{PhysicsSnapshot, Snapshot},
  },
  bevy::{app::AppLabel, ecs::schedule::ScheduleLabel},
  harness::Harness,
  std::time::Duration,
};

#[derive(ScheduleLabel, Debug, Hash, PartialEq, Eq, Clone, Default)]
pub struct Step;

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, AppLabel)]
struct FluidApp;

pub fn plugin(app: &mut App, harness: Harness, fluids: Fluids) {
  let mut sub_app = SubApp::new();
  sub_app.update_schedule = Some(Step.intern());
  sub_app.init_schedule(Main.intern());

  sub_app.set_extract(|main, sub| {
    if let Some(FrameCell(frame)) = sub.remove_resource::<FrameCell>()
      && let Some(mut timeline) = main.get_resource_mut::<Timeline>()
    {
      timeline.snapshots.push(frame);
    }
  });
  sub_app.world_mut().insert_non_send_resource(harness);
  sub_app.world_mut().insert_non_send_resource(fluids);
  sub_app
    .init_resource::<Time<Sim>>()
    .insert_resource(Wait(Timer::from_seconds(1.0, TimerMode::Once)))
    .add_systems(Step, (tick::setup, step, tick::update).chain());

  app.insert_sub_app(FluidApp, sub_app);
  app.init_resource::<Timeline>().add_systems(Update, draw);
}

#[derive(Default)]
struct Sim;

#[derive(Resource, Deref, DerefMut)]
struct Wait(Timer);

fn step(
  harness: NonSendMut<Harness>,
  mut fluids: NonSendMut<Fluids>,
  mut time: ResMut<Time<Sim>>,
  mut commands: Commands,
) {
  let harness = harness.into_inner();
  {
    let physics = PhysicsSnapshot::capture(harness);
    let fluids = fluids.snapshot();
    commands.insert_resource(FrameCell((physics, fluids)));
  }
  harness.step();
  fluids.step(&mut harness.physics, &harness.state);

  let delta = harness.physics.integration_parameters.dt;
  time.advance_by(Duration::from_secs_f32(delta));
}

#[derive(Resource)]
pub struct FrameCell(Frame);

#[derive(Resource, Default)]
pub struct Timeline {
  snapshots: Vec<Frame>,
  timestamp: usize,
}

impl Timeline {
  pub fn step(&mut self) -> Option<&Frame> {
    if let Some(snapshot) = self.snapshots.get(self.timestamp) {
      if self.timestamp != self.snapshots.len() - 1 {
        self.timestamp += 1;
      }
      Some(snapshot)
    } else {
      None
    }
  }
}

pub type Frame = (PhysicsSnapshot, FluidsSnapshot);

fn draw(
  mut gizmos: Gizmos,
  mut timeline: ResMut<Timeline>,
  input: Res<ButtonInput<KeyCode>>,
) {
  if input.just_pressed(KeyCode::Space) {
    timeline.timestamp = 0;
  }

  if let Some((_physics, fluids)) = timeline.step() {
    // physics.draw(&mut gizmos);
    fluids.draw(&mut gizmos);
  }
}
