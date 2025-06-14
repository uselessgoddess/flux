use {
  crate::{
    harness::{Fluids, FluidsSnapshot, Plugin},
    prelude::*,
    snapshot::{PhysicsSnapshot, Snapshot},
  },
  crossbeam::{channel, channel::Receiver},
  harness::Harness,
};

pub struct Stand {
  harness: Harness,
  fluids: Fluids,
}

impl Stand {
  pub fn new(harness: Harness, fluids: Fluids) -> Self {
    Self { harness, fluids }
  }

  pub fn detach_thread(self) -> Receiver<Frame> {
    let (tx, rx) = channel::bounded(32);

    let Self { mut harness, mut fluids } = self;
    std::thread::spawn(move || {
      loop {
        {
          let physics = PhysicsSnapshot::capture(&harness);
          let fluids = fluids.snapshot();
          let _ = tx.send((physics, fluids));
        }
        harness.step();
        fluids.step(&mut harness.physics, &harness.state);
      }
    });
    rx
  }

  pub fn plugin(app: &mut App, rx: Receiver<Frame>) {
    app
      .init_resource::<Timeline>()
      .insert_resource(SnapshotSteam(rx))
      .add_systems(Update, (receive, draw));
  }
}

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

type Frame = (PhysicsSnapshot, FluidsSnapshot);

#[derive(Resource, Deref)]
struct SnapshotSteam(Receiver<Frame>);

fn receive(mut timeline: ResMut<Timeline>, rx: Res<SnapshotSteam>) {
  for snapshot in rx.try_iter() {
    timeline.snapshots.push(snapshot);
  }
}

fn draw(
  mut gizmos: Gizmos,
  mut timeline: ResMut<Timeline>,
  input: Res<ButtonInput<KeyCode>>,
) {
  if input.just_pressed(KeyCode::Space) {
    timeline.timestamp = 0;
  }

  if let Some((physics, fluids)) = timeline.step() {
    physics.draw(&mut gizmos);
    fluids.draw(&mut gizmos);
  }
}
