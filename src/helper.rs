use salva::{
  math::{Point, Vector},
  object::Fluid,
};

pub fn cube_fluid(
  ni: usize,
  nj: usize,
  nk: usize,
  particle_rad: f32,
  density: f32,
) -> Fluid {
  let mut points = Vec::new();
  let half_extents =
    Vector::new(ni as f32, nj as f32, nk as f32) * particle_rad;

  for i in 0..ni {
    for j in 0..nj {
      for k in 0..nk {
        let x = (i as f32) * particle_rad * 2.0;
        let y = (j as f32) * particle_rad * 2.0;
        let z = (k as f32) * particle_rad * 2.0;
        points.push(
          Point::new(x, y, z) + Vector::repeat(particle_rad) - half_extents,
        );
      }
    }
  }

  use salva::object::interaction_groups::InteractionGroups;

  Fluid::new(points, particle_rad, density, InteractionGroups::default())
}
