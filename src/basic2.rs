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

// Soap grid.
const SOAP_COLS: usize = 40;
const SOAP_ROWS: usize = 12;
const SOAP_CELL_HALF: f32 = 0.025;
const SOAP_CELL_SIZE: f32 = SOAP_CELL_HALF * 2.0;
const SOAP_MIN_HALF: f32 = 0.005;
// How fast a wet cell shrinks (half-extent lost per second of contact). Slow on purpose.
const SOAP_SHRINK_RATE: f32 = 0.003;
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
}

/// Break a cell off the parent soap body: swap its collider/boundary for a
/// fresh pair on a brand new dynamic body placed at the cell's current pose.
fn detach_soap_cell(
    cell: &mut SoapCell,
    world: &mut PhysicsWorld,
    fluids_pipeline: &mut FluidsPipeline,
) {
    let Some(collider) = world.colliders.get(cell.collider_handle) else {
        return;
    };
    let world_pos = *collider.position();
    // Shrink a hair on detach: the cell was tiling edge-to-edge with its still-attached
    // neighbors (zero clearance), and spawning a brand new dynamic body exactly touching
    // them is a classic way to get a tiny floating-point overlap that the contact solver
    // "resolves" by launching the new body off-screen in one step. A small margin avoids it.
    let half_extent = cell.half_extent * 0.9;

    fluids_pipeline.coupling.unregister_coupling(cell.collider_handle);
    fluids_pipeline.liquid_world.remove_boundary(cell.boundary_handle);
    world.colliders.remove(
        cell.collider_handle,
        &mut world.islands,
        &mut world.bodies,
        true,
    );

    let new_body = RigidBodyBuilder::dynamic()
        .pose(world_pos)
        .linvel(Vector2::zeros().into())
        .angvel(0.0)
        .build();
    let new_body_handle: RigidBodyHandle = world.bodies.insert(new_body);
    let new_collider = ColliderBuilder::cuboid(half_extent, half_extent).build();
    let new_co_handle =
        world
            .colliders
            .insert_with_parent(new_collider, new_body_handle, &mut world.bodies);

    let new_bo_handle = fluids_pipeline
        .liquid_world
        .add_boundary(Boundary::new(Vec::new(), InteractionGroups::default()));
    fluids_pipeline.coupling.register_coupling(
        new_bo_handle,
        new_co_handle,
        ColliderSampling::DynamicContactSampling,
    );

    cell.collider_handle = new_co_handle;
    cell.boundary_handle = new_bo_handle;
    cell.half_extent = half_extent;
    cell.attached = false;
}

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    /*
     * World
     */
    let mut world = PhysicsWorld::new();
    world.gravity = (Vector2::y() * -20.81).into();
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
     * Ground
     */
    let ground_size = Vector2::new(10.0, 1.0);
    let nsubdivs = 50;

    let heights: Vec<_> = (0..=nsubdivs)
        .map(|i| {
            if i == 0 || i == nsubdivs {
                20.0
            } else {
                0.0
                //(i as f32 * ground_size.x / (nsubdivs as f32)).cos() * 0.5
            }
        })
        .collect();

    let rigid_body = RigidBodyBuilder::fixed().build();
    let handle = world.bodies.insert(rigid_body);
    let collider = ColliderBuilder::heightfield(heights, ground_size.into()).build();
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
        .translation(Vector2::new(0.0, 0.5).into())
        .lock_translations()
        .lock_rotations()
        .build();
    let soap_body_handle = world.bodies.insert(soap_body);

    let mut soap_cells = Vec::new();
    let soap_half_width = SOAP_COLS as f32 * SOAP_CELL_HALF;
    let soap_half_height = SOAP_ROWS as f32 * SOAP_CELL_HALF;

    for row in 0..SOAP_ROWS {
        for col in 0..SOAP_COLS {
            let local_x = -soap_half_width + SOAP_CELL_HALF + col as f32 * SOAP_CELL_SIZE;
            let local_y = -soap_half_height + SOAP_CELL_HALF + row as f32 * SOAP_CELL_SIZE;

            let collider = ColliderBuilder::cuboid(SOAP_CELL_HALF, SOAP_CELL_HALF)
                .translation(Vector2::new(local_x, local_y).into())
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
                half_extent: SOAP_CELL_HALF,
                wet_time: 0.0,
                attached: true,
            });
        }
    }

    /*
     * Set up the viewer and run the simulation.
     */
    plugin.set_pipeline(fluids_pipeline);
    viewer.set_world(&mut world);
    viewer.look_at(Vector2::new(0.0, 5.5).into(), 50.0);

    let mut particle_spawn_positions = Vec::new();
    particle_spawn_positions.push(Vector2::new(-2.5, 10.0));
    let mut velocities = Vec::new();
    velocities.push(Vector2::new(2.0, -2.0));

    let mut rng = rand::rng();
    let mut steps = 0;
    while viewer.render_frame(&mut world).await {
        plugin.update_from_settings(viewer.example_settings_mut());
        plugin.draw(viewer);

        if viewer.simulating() {
            if steps % 20 == 0 {
                let fl = plugin
                    .pipeline_mut()
                    .liquid_world
                    .fluids_mut()
                    .get_mut(fluid_handle)
                    .unwrap();
                fl.add_particles(&particle_spawn_positions, Some(&velocities));
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

                // Wet cells slowly shrink.
                cell.wet_time += dt;
                if cell.half_extent > SOAP_MIN_HALF {
                    cell.half_extent = (cell.half_extent - SOAP_SHRINK_RATE * dt).max(SOAP_MIN_HALF);
                    if let Some(collider) = world.colliders.get_mut(cell.collider_handle) {
                        collider.set_shape(SharedShape::cuboid(cell.half_extent, cell.half_extent));
                    }
                }

                // Chance to snap off the parent group, scaled by force and time.
                if cell.attached {
                    let decouple_chance = force.norm() * SOAP_DECOUPLE_COEFF * dt;
                    if rng.random::<f32>() < decouple_chance {
                        detach_soap_cell(cell, &mut world, plugin.pipeline_mut());
                    }
                }
            }
        }
        steps += 1;
    }

    Ok(())
}
