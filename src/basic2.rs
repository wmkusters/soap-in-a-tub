extern crate nalgebra as na;

use na::{Vector2, Vector3};
use rand::RngExt;
use rapier2d::dynamics::{RigidBodyBuilder, RigidBodyHandle};
use rapier2d::geometry::{ColliderBuilder, ColliderHandle, SharedShape};
use rapier2d::pipeline::PhysicsWorld;
use rapier_testbed2d::TestbedViewer;
use salva2d::integrations::rapier::{ColliderSampling, FluidsPipeline, FluidsTestbedPlugin};
use salva2d::object::interaction_groups::InteractionGroups;
use salva2d::object::{Boundary, BoundaryHandle, Fluid};
use salva2d::solver::XSPHViscosity;
use std::f32;

const PARTICLE_RADIUS: f32 = 0.1;
const SMOOTHING_FACTOR: f32 = 2.0;

const NUM_PARTICLES_SPAWN: usize = 2;

// Soap grid, sized in meters. Rows/cols are derived (see `soap_grid_dims`) from these
// plus a target cell count, keeping cells square.
const SOAP_WIDTH: f32 = 4.0;
const SOAP_HEIGHT: f32 = 1.2;
const SOAP_NUM_CELLS: usize = 480;
const SOAP_MIN_HALF: f32 = 0.01;
// Water in this sim has density 1.0 (see Fluid::new below); real soap runs slightly
// denser (~1.1), so this is enough to make it sink rather than float.
const SOAP_DENSITY: f32 = 1.1;
// How fast a wet cell shrinks (half-extent lost per second of contact). Slow on purpose.
const SOAP_SHRINK_RATE: f32 = 0.003;
// Free-floating (detached) cells dissolve much faster than ones still attached to the bar.
const SOAP_FREE_SHRINK_MULTIPLIER: f32 = 10.0;
// Decouple probability per second is (force magnitude * this coefficient).
const SOAP_DECOUPLE_COEFF: f32 = 0.05;

/// One square chunk of the soap bar. Starts parented to the shared soap body;
/// once decoupled it gets its own free-standing body.
struct SoapCell {
    collider_handle: ColliderHandle,
    boundary_handle: BoundaryHandle,
    half_extent: f32,
    wet_time: f32,
    attached: bool,
    dissolved: bool,
}

/// Fully erode a cell once it shrinks past `SOAP_MIN_HALF`: drop its salva coupling/boundary
/// and remove it from the physics world. An attached cell only loses its own collider (the
/// shared soap body and its other cells stay put); a detached cell's private body is removed
/// too, since nothing else references it.
fn dissolve_soap_cell(
    cell: &mut SoapCell,
    world: &mut PhysicsWorld,
    fluids_pipeline: &mut FluidsPipeline,
) {
    fluids_pipeline.coupling.unregister_coupling(cell.collider_handle);
    fluids_pipeline.liquid_world.remove_boundary(cell.boundary_handle);

    if cell.attached {
        world.colliders.remove(
            cell.collider_handle,
            &mut world.islands,
            &mut world.bodies,
            true,
        );
    } else if let Some(body_handle) = world
        .colliders
        .get(cell.collider_handle)
        .and_then(|c| c.parent())
    {
        world.bodies.remove(
            body_handle,
            &mut world.islands,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
            true,
        );
    }

    cell.dissolved = true;
}

/// Break a cell off the parent soap body by reparenting its *existing* collider onto a
/// brand new free-standing dynamic body, in place.
///
/// Earlier version removed the old collider and inserted a fresh one, which turned out to
/// be why detached cells went invisible: rapier_testbed2d's GraphicsManager only registers
/// render nodes for colliders it saw at `set_world()` (or a full RESET_WORLD_GRAPHICS
/// rescan) — a collider inserted later via raw `insert_with_parent` never gets a node, so
/// it's still simulated (we confirmed via velocity logging it wasn't being launched) but
/// never drawn. Reparenting keeps the same `ColliderHandle`, which the renderer already has
/// a node for (it re-reads that handle's live position every frame), and also keeps salva's
/// existing coupling/boundary registration valid, since that's keyed on the same handle too.
fn detach_soap_cell(cell: &mut SoapCell, world: &mut PhysicsWorld) {
    let Some(collider) = world.colliders.get(cell.collider_handle) else {
        return;
    };
    let world_pos = *collider.position();

    let new_body = RigidBodyBuilder::dynamic()
        .pose(world_pos)
        .linvel(Vector2::zeros().into())
        .angvel(0.0)
        .build();
    let new_body_handle: RigidBodyHandle = world.bodies.insert(new_body);

    world
        .colliders
        .set_parent(cell.collider_handle, Some(new_body_handle), &mut world.bodies);

    // `set_parent` keeps the collider's old local offset relative to its *previous* parent
    // (that's the whole grid-cell offset within the old soap body) — re-zero it now that the
    // new body already sits exactly at the collider's current world pose, or it'll jump.
    if let Some(collider) = world.colliders.get_mut(cell.collider_handle) {
        collider.set_position_wrt_parent(na::Isometry2::identity().into());
    }

    cell.attached = false;
}

/// Pick a (cols, rows, cell_half_extent) grid that packs roughly `target_cells` square
/// cells into a `width` x `height` bar. Cols are sized to fit `width` exactly; rows only
/// approximate `height`, since cell count is an integer and cells must stay square.
fn soap_grid_dims(width: f32, height: f32, target_cells: usize) -> (usize, usize, f32) {
    let cell_size = (width * height / target_cells as f32).sqrt();
    let cols = (width / cell_size).round().max(1.0) as usize;
    let rows = (height / cell_size).round().max(1.0) as usize;
    let cell_half = width / cols as f32 / 2.0;
    (cols, rows, cell_half)
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();
    world.gravity = (Vector2::y() * -9.81).into();
    world.integration_parameters.dt = 1.0 / 200.0;

    let mut plugin = FluidsTestbedPlugin::new();
    let mut fluids_pipeline = FluidsPipeline::new(PARTICLE_RADIUS, SMOOTHING_FACTOR);

    // Liquid.
    let viscosity = XSPHViscosity::new(0.1, 0.5);
    let mut fluid = Fluid::new(Vec::new(), PARTICLE_RADIUS, 1.0, InteractionGroups::default());
    fluid.nonpressure_forces.push(Box::new(viscosity.clone()));
    let fluid_handle = fluids_pipeline.liquid_world.add_fluid(fluid);
    plugin.set_fluid_color(fluid_handle, Vector3::new(0.6, 0.8, 0.5));

    /*
     * Ground: an enclosing tub built from one compound collider (floor + two side
     * walls) on a single fixed body, instead of a heightfield. The old heightfield
     * faked walls by spiking its end cells up to y=20 over one narrow segment (a
     * steep cliff, not an actual vertical wall); real cuboids give true vertical
     * sides. Open at the top, since particles are spawned falling in from above.
     */
    let tub_width = 20.0;
    let tub_height = 10.0;
    let wall_thickness = 0.5;
    let floor_y = 0.0;
    let floor_half_thickness = wall_thickness / 2.0;

    let tub_shapes = vec![
        (
            na::Isometry2::translation(0.0, floor_y - floor_half_thickness).into(),
            SharedShape::cuboid(tub_width / 2.0 + wall_thickness, floor_half_thickness),
        ),
        (
            na::Isometry2::translation(
                -tub_width / 2.0 - wall_thickness / 2.0,
                floor_y + tub_height / 2.0,
            )
            .into(),
            SharedShape::cuboid(wall_thickness / 2.0, tub_height / 2.0),
        ),
        (
            na::Isometry2::translation(
                tub_width / 2.0 + wall_thickness / 2.0,
                floor_y + tub_height / 2.0,
            )
            .into(),
            SharedShape::cuboid(wall_thickness / 2.0, tub_height / 2.0),
        ),
    ];

    let rigid_body = RigidBodyBuilder::fixed().build();
    let handle = world.bodies.insert(rigid_body);
    let collider = ColliderBuilder::new(SharedShape::compound(tub_shapes)).build();
    let co_handle = world
        .colliders
        .insert_with_parent(collider, handle, &mut world.bodies);
    let bo_handle = fluids_pipeline
        .liquid_world
        .add_boundary(Boundary::new(Vec::new(), InteractionGroups::default()));
    fluids_pipeline.coupling.register_coupling(
        bo_handle,
        co_handle,
        ColliderSampling::DynamicContactSampling,
    );

    /*
     * Soap: a rectangular grid of small square cells, each its own collider
     * parented to one shared body. Must be `dynamic` (motion locked) rather
     * than `fixed` — salva only tracks per-cell forces on a coupled boundary
     * when its parent body is dynamic (see fluids_pipeline.rs::update_boundaries).
     */
    let soap_body = RigidBodyBuilder::dynamic()
        .translation(Vector2::new(0.0, 0.61).into())
        .lock_translations()
        .lock_rotations()
        .build();
    let soap_body_handle = world.bodies.insert(soap_body);

    let (soap_cols, soap_rows, soap_cell_half) =
        soap_grid_dims(SOAP_WIDTH, SOAP_HEIGHT, SOAP_NUM_CELLS);
    let soap_cell_size = soap_cell_half * 2.0;

    let mut soap_cells = Vec::new();
    let soap_half_width = soap_cols as f32 * soap_cell_half;
    let soap_half_height = soap_rows as f32 * soap_cell_half;

    for row in 0..soap_rows {
        for col in 0..soap_cols {
            let local_x = -soap_half_width + soap_cell_half + col as f32 * soap_cell_size;
            let local_y = -soap_half_height + soap_cell_half + row as f32 * soap_cell_size;

            let collider = ColliderBuilder::cuboid(soap_cell_half, soap_cell_half)
                .translation(Vector2::new(local_x, local_y).into())
                .density(SOAP_DENSITY)
                .build();
            let co_handle =
                world
                    .colliders
                    .insert_with_parent(collider, soap_body_handle, &mut world.bodies);

            let bo_handle = fluids_pipeline
                .liquid_world
                .add_boundary(Boundary::new(Vec::new(), InteractionGroups::default()));
            fluids_pipeline.coupling.register_coupling(
                bo_handle,
                co_handle,
                ColliderSampling::DynamicContactSampling,
            );

            soap_cells.push(SoapCell {
                collider_handle: co_handle,
                boundary_handle: bo_handle,
                half_extent: soap_cell_half,
                wet_time: 0.0,
                attached: true,
                dissolved: false,
            });
        }
    }

    /*
     * Set up the viewer and run the simulation.
     */
    plugin.set_pipeline(fluids_pipeline);
    viewer.set_world(&mut world);
    viewer.look_at(Vector2::new(0.0, 5.5).into(), 50.0);

    // let mut particle_spawn_positions = Vec::new();
    // particle_spawn_positions.push(Vector2::new(-2.5, 10.0));
    // let mut velocities = Vec::new();
    // velocities.push(Vector2::new(2.0, -2.0));

    let mut rng = rand::rng();
    let mut steps = 0;
    while viewer.render_frame(&mut world).await {
        plugin.update_from_settings(viewer.example_settings_mut());
        plugin.draw(viewer);

        if viewer.simulating() {
            if steps % 5 == 0 {
                let fl = plugin
                    .pipeline_mut()
                    .liquid_world
                    .fluids_mut()
                    .get_mut(fluid_handle)
                    .unwrap();
                let (particle_spawn_positions, velocities) = spawn_particles(NUM_PARTICLES_SPAWN);
                fl.add_particles(&particle_spawn_positions, Some(&velocities));
                let so_pos = world.bodies.get(soap_body_handle).unwrap().translation();
            }
            world.step();
            plugin.step(&mut world);

            let dt = world.integration_parameters.dt;
            for cell in &mut soap_cells {
                let contact_force = {
                    let liquid_world = &plugin.pipeline_mut().liquid_world;
                    liquid_world
                        .boundaries()
                        .get(cell.boundary_handle)
                        .filter(|b| !b.positions.is_empty())
                        .and_then(|b| b.forces.as_ref())
                        .map(|forces| {
                            forces
                                .read()
                                .unwrap()
                                .iter()
                                .fold(Vector2::zeros(), |acc, f| acc + *f)
                        })
                };

                let Some(force) = contact_force else {
                    continue;
                };

                // Wet cells slowly shrink; detached ones dissolve much faster. Once a cell
                // erodes past SOAP_MIN_HALF it's removed from the sim entirely.
                cell.wet_time += dt;
                let rate = if cell.attached {
                    SOAP_SHRINK_RATE
                } else {
                    SOAP_SHRINK_RATE * SOAP_FREE_SHRINK_MULTIPLIER
                };
                cell.half_extent -= rate * dt;
                if cell.half_extent <= SOAP_MIN_HALF {
                    dissolve_soap_cell(cell, &mut world, plugin.pipeline_mut());
                    continue;
                }
                if let Some(collider) = world.colliders.get_mut(cell.collider_handle) {
                    collider.set_shape(SharedShape::cuboid(cell.half_extent, cell.half_extent));
                }

                // Chance to snap off the parent group, scaled by force and time.
                if cell.attached {
                    let decouple_chance = force.norm() * SOAP_DECOUPLE_COEFF * dt;
                    if rng.random::<f32>() < decouple_chance {
                        detach_soap_cell(cell, &mut world);
                    }
                }
            }
            soap_cells.retain(|cell| !cell.dissolved);

            if steps % 20 == 0 {
                let mut detached_count = 0;
                let mut fastest: Option<(usize, f32)> = None;
                for (i, cell) in soap_cells.iter().enumerate() {
                    if cell.attached {
                        continue;
                    }
                    detached_count += 1;
                    let speed = world
                        .colliders
                        .get(cell.collider_handle)
                        .and_then(|c| c.parent())
                        .and_then(|body_handle| world.bodies.get(body_handle))
                        .map(|body| {
                            let v = body.linvel();
                            (v.x * v.x + v.y * v.y).sqrt()
                        })
                        .unwrap_or(0.0);
                    if fastest.map_or(true, |(_, s)| speed > s) {
                        fastest = Some((i, speed));
                    }
                }
                if let Some((i, speed)) = fastest {
                    println!(
                        "detached cells: {detached_count}, fastest: cell {i} at {speed:.2} m/s"
                    );
                }
            }
        }
        steps += 1;
    }

    Ok(())
}

fn spawn_particles(n: usize) -> (Vec<Vector2<f32>>, Vec<Vector2<f32>>) {
    let mut rng = rand::rng();
    let mut particle_spawn_positions = Vec::new();
    let mut velocities = Vec::new();
    for _ in 0..n {
        let p_jitter = rng.random_range(-0.5..0.5);
        particle_spawn_positions.push(Vector2::new(-2.5 + p_jitter, 10.0 + p_jitter));
        velocities.push(Vector2::new(2.0, -8.0));
    }
    return (particle_spawn_positions, velocities)
}
