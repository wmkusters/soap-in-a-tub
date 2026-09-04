#![allow(dead_code)]
#![allow(clippy::type_complexity)]

use rapier_testbed2d::{ExampleEntry, TestbedViewer};

mod basic2;

#[kiss3d::main]
pub async fn main() {
    let mut viewer = TestbedViewer::new(vec![ExampleEntry::new("Fluids", "Basic")]).await;

    // basic2 owns its physics state and render loop; this outer loop just
    // re-runs it after a UI-triggered restart.
    loop {
        viewer.clear_scene();
        if let Err(e) = basic2::run(&mut viewer).await {
            eprintln!("basic2 example failed: {e:?}");
        }
        if viewer.quitting() {
            break;
        }
    }
}
