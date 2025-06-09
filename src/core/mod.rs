mod debug;
mod physics;

pub use crate::prelude::*;

pub fn plugin(app: &mut App) {
  app.add_plugins(physics::plugin);

  app.add_plugins(debug::plugin);
}
