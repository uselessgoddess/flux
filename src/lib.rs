#![feature(let_chains)]
extern crate nalgebra as na;

mod core;
mod fluid;
pub mod stand;

pub use prelude::*;

pub mod prelude {
  pub use {
    crate::{
      core::*,
      fluid::{FluidCallback, FluidsPlugin},
      stand::{self, Stand},
    },
    bevy::prelude::*,
    panorbit_camera::{
      PanOrbitCamera, PanOrbitCameraPlugin, PanOrbitCameraSystemSet,
    },
  };
}

pub fn app() -> App {
  let mut app = App::new();
  app
    .add_plugins(DefaultPlugins)
    .add_plugins(core::plugin)
    .add_plugins(PanOrbitCameraPlugin);
  app
}
