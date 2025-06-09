use crate::prelude::*;

pub fn plugin(app: &mut App) {
  app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());
}
