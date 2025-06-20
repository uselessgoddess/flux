use {
  parry::{bounding_volume::Aabb, shape::Shape},
  salva::{
    LiquidWorld,
    math::{Point, Vector},
    object::FluidHandle,
    parry, sampling,
  },
};

pub struct ShapeFlow {
  center: Vector<f32>,
  velocity: Vector<f32>,
  samples: Vec<Point<f32>>,
  radius: f32,
}

fn ball_aabb(center: Point<f32>, radius: f32) -> Aabb {
  Aabb::new(center + Vector::repeat(-radius), center + Vector::repeat(radius))
}

impl ShapeFlow {
  pub fn new<S: Shape>(
    center: Vector<f32>,
    shape: &S,
    radius: f32,
  ) -> Option<Self> {
    let samples = sampling::shape_volume_ray_sample(shape, radius)?;
    Some(Self { center, samples, radius, velocity: Default::default() })
  }

  pub fn with_velocity(mut self, velocity: Vector<f32>) -> Self {
    self.velocity = velocity;
    self
  }

  pub fn emit(&self, world: &mut LiquidWorld, handle: FluidHandle) -> bool {
    let particles =
      self.samples.iter().map(|&sample| sample + self.center).flat_map(
        |sample| {
          let aabb = ball_aabb(sample, self.radius);
          if world.particles_intersecting_aabb(aabb).count() == 0 {
            Some(sample)
          } else {
            None
          }
        },
      );

    let particles: Vec<_> = particles.collect();
    let Some(fluid) = world.fluids_mut().get_mut(handle) else { return false };

    let velocities: Vec<_> = particles.iter().map(|_| self.velocity).collect();
    fluid.add_particles(&particles, Some(&velocities));

    true
  }
}
