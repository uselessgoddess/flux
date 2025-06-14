#![feature(let_chains)]
extern crate nalgebra as na;

mod core;
pub mod harness;
pub mod snapshot;
pub mod stand;

pub use prelude::*;

pub mod prelude {
  pub use {
    crate::{
      core::*,
      harness, snapshot,
      stand::{self, Stand},
    },
    bevy::prelude::*,
    panorbit_camera::{
      PanOrbitCamera, PanOrbitCameraPlugin, PanOrbitCameraSystemSet,
    },
  };

  pub type Real = f32;
}

pub fn app() -> App {
  let mut app = App::new();
  app
    .add_plugins(DefaultPlugins)
    .add_plugins(core::plugin)
    .add_plugins(PanOrbitCameraPlugin);
  app
}
