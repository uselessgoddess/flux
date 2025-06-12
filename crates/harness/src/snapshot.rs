use std::{
  any::{Any, TypeId},
  ops::{Deref, DerefMut},
};

pub struct SharedSnapshot(Box<dyn Any + Sync + Send>);

impl Deref for SharedSnapshot {
  type Target = Box<dyn Snapshot>;

  fn deref(&self) -> &Self::Target {
    self.0.downcast_ref().unwrap()
  }
}

impl SharedSnapshot {
  pub fn new(snapshot: impl Snapshot + Sync + Send + 'static) -> Self {
    SharedSnapshot(Box::new(snapshot))
  }

  pub fn downcast<T: Snapshot>(&self) -> Option<&T> {
    self.0.downcast_ref()
  }
}

pub trait Snapshot: Any {}
