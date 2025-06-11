use {
  crate::prelude::*,
  harness::{Harness, PhysicsState},
};

pub trait Plugin: harness::Plugin {
  fn draw(&mut self, gizmos: &mut Gizmos, harness: &mut Harness);
}

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

  pub fn plugin(self, app: &mut App) {
    let Self { harness, plugins } = self;
    app
      .insert_non_send_resource(harness)
      .insert_non_send_resource(plugins)
      .add_systems(Update, draw)
      .add_systems(FixedUpdate, step);
  }
}

fn step(
  harness: NonSendMut<Harness>,
  mut plugins: NonSendMut<Plugins>,
  input: Res<ButtonInput<KeyCode>>,
) {
  let harness = harness.into_inner();

  if input.pressed(KeyCode::Space) {
    harness.step();

    for plugin in plugins.iter_mut() {
      plugin.step(&mut harness.physics, &harness.state);
    }
  }
}

fn draw(
  mut gizmos: Gizmos,
  harness: NonSendMut<Harness>,
  mut plugins: NonSendMut<Plugins>,
) {
  let harness = harness.into_inner();

  draw_physics(&mut gizmos, &harness.physics);

  for plugin in plugins.iter_mut() {
    plugin.draw(&mut gizmos, harness);
  }
}

fn draw_physics(gizmos: &mut Gizmos, physics: &PhysicsState) {
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
            ..default()
          },
          Color::srgb(1.0, 0.0, 0.0),
        );
      }
    }
  }
}
