mod core;

pub use prelude::*;

pub mod prelude {
  pub use {bevy::prelude::*, core::*, rapier3d::prelude::*};
}

pub fn app() -> App {
  let mut app = App::new();
  app.add_plugins(DefaultPlugins).add_plugins(core::plugin);
  app
}
