extern crate nalgebra as na;

use na::{Vector2, Vector3};
use rapier2d::dynamics::RigidBodyBuilder;
use rapier2d::geometry::{Collider, ColliderBuilder};
use rapier2d::pipeline::PhysicsWorld;
use rapier_testbed2d::TestbedViewer;
use salva2d::integrations::rapier::{ColliderSampling, FluidsPipeline, FluidsTestbedPlugin};
use salva2d::object::interaction_groups::InteractionGroups;
use salva2d::object::{Boundary, Fluid};
use salva2d::solver::{XSPHViscosity};
use std::f32;

const PARTICLE_RADIUS: f32 = 0.1;
const SMOOTHING_FACTOR: f32 = 2.0;

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
     * Soap (fixed rigid body, stuck to the floor).
     */
    let soap_collider = ColliderBuilder::cuboid(1.0, 0.3).build();
    let soap_samples =
        salva2d::sampling::shape_surface_ray_sample(soap_collider.shape(), PARTICLE_RADIUS)
            .unwrap();

    let soap_body = RigidBodyBuilder::fixed()
        .translation(Vector2::new(0.0, 0.5).into())
        .build();
    let soap_body_handle = world.bodies.insert(soap_body);
    let soap_co_handle =
        world
            .colliders
            .insert_with_parent(soap_collider, soap_body_handle, &mut world.bodies);
    let soap_bo_handle = fluids_pipeline
        .liquid_world
        .add_boundary(Boundary::new(Vec::new(), InteractionGroups::default()));
    fluids_pipeline.coupling.register_coupling(
        soap_bo_handle,
        soap_co_handle,
        ColliderSampling::StaticSampling(soap_samples),
    );


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

    let mut steps = 0;
    while viewer.render_frame(&mut world).await {
        plugin.update_from_settings(viewer.example_settings_mut());
        plugin.draw(viewer);

        if viewer.simulating() {
            if steps % 20 == 0 {
                let fl = plugin.pipeline_mut().liquid_world.fluids_mut().get_mut(fluid_handle).unwrap();
                fl.add_particles(&particle_spawn_positions, Some(&velocities));
            }
            world.step();
            plugin.step(&mut world);
        }
        steps += 1;
    }

    Ok(())
}
