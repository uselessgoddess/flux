use {
  crate::prelude::*,
  crossbeam::{channel, channel::Receiver},
  harness::{Harness, PhysicsState, SharedSnapshot},
};

pub trait Plugin: harness::Plugin + Send {}

#[derive(Deref, DerefMut)]
struct Plugins(Vec<Box<dyn Plugin>>);

pub struct Stand {
  harness: Harness,
  plugins: Plugins,
}

impl Stand {
  pub fn new(harness: Harness) -> Self {
    Self { harness, plugins: Plugins(Vec::new()) }
  }

  pub fn add_plugin(mut self, plugin: impl Plugin + 'static) -> Self {
    self.plugins.push(Box::new(plugin));
    self
  }

  pub fn detach_thread(self) -> Receiver<Vec<SharedSnapshot>> {
    let (tx, rx) = channel::bounded(32);

    let Self { mut harness, mut plugins } = self;
    std::thread::spawn(move || {
      loop {
        let snapshot = plugins.iter().map(|p| p.snapshot()).collect();
        let _ = tx.send(snapshot);

        harness.step();

        for plugin in plugins.iter_mut() {
          plugin.step(&mut harness.physics, &harness.state);
        }
      }
    });
    rx
  }

  pub fn plugin(app: &mut App, rx: Receiver<Vec<SharedSnapshot>>) {
    app
      .init_resource::<Timeline>()
      .insert_resource(SnapshotSteam(rx))
      .add_systems(Update, (receive, draw));
  }
}

#[derive(Resource, Default)]
pub struct Timeline {
  snapshots: Vec<Vec<SharedSnapshot>>,
  timestamp: usize,
}

impl Timeline {
  pub fn step(&mut self) -> Option<&[SharedSnapshot]> {
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

#[derive(Resource, Deref)]
struct SnapshotSteam(Receiver<Vec<SharedSnapshot>>);

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

  if let Some(snapshots) = timeline.step() {
    for snapshot in snapshots {
      draw_fluids(snapshot, &mut gizmos);
    }
  }
}

fn draw_fluids(snapshot: &SharedSnapshot, gizmos: &mut Gizmos) {
  let Some(crate::fluid::Snapshot { fluids, particle_radius }) =
    snapshot.downcast()
  else {
    return;
  };

  for (_, fluid) in fluids.iter() {
    for particle in &fluid.positions {
      gizmos.sphere(
        Isometry3d::from_translation(Vec3::from_slice(
          particle.coords.as_slice(),
        )),
        *particle_radius,
        Color::srgb(0.0, 0.2, 0.65),
      );
    }
  }
}

fn _draw_physics(gizmos: &mut Gizmos, physics: &PhysicsState) {
  for (_, body) in physics.bodies.iter() {
    for &handle in body.colliders() {
      if let Some(collider) = physics.colliders.get(handle)
        && let Some(cuboid) = collider.shape().as_cuboid()
      {
        let pos = body.translation();
        let rot = body.rotation();
        gizmos.cuboid(
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
