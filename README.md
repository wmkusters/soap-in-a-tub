## Soap in the Tub
An attempt at modeling the time evolution of a bar of soap in a tub. The project is written in Rust using the [Rapier](https://rapier.rs/) and [Salva](https://salva.rs/) physics libraries. 

## Building and running
I have only tested building this project on my M2 Mac using Rust's `cargo` tool. A simple Makefile is included and can be used to run the project if `cargo` is installed via `make run`. If built successfully, you should see a window popup to view the simulation visualizer.

[Please see this page to install Cargo](https://doc.rust-lang.org/cargo/getting-started/installation.html). 

## Project Overview

The simulation focuses on modeling two processes: mechanical erosion and dissolution. Both processes are grossly simplified. The soap itself is modeled as a set of separate but bound cells, each represented by a Rapier `collider` object. These cells are bound to a larger parent body that has translation and rotation locked. This allows for forces upon individual cells to be calculated while the soap "body" remains stuck in place. 

Water particles are created at a constant step rate throughout the simulation. These particles fall from a height and impact the soap. The generated forces have a chance to break off an individual soap cell, at which point the soap cell is separated from the parent body. 

Dissolution occurs whenever a force from the fluid is acting upon a soap cell. This is modeled as a constant rate of shrinkage in the soap cell size. Soap cells that are detached from the parent soap body shrink at 10x the rate of the cells that are still attached. Once below a certain size threshold, the soap cell is deleted from the simulation.

Visualization is managed via Rapier's `TestbedViewer`. 

### Other notes

The physical values are not represenative of a realistic system. The simulation was behaving much better with larger bodies, likely due to the relative size of cells and the water particles.

## Focus Area

I particularly enjoyed the mechanical decoupling of individual soap cells. I think in aggregate it causes the simulation to look very natural. It required a fair bit of constant tuning to get it to behave quite right, but it results in soap deformation that feels intuitive. Using Rapier worked well here, as the hierarchical relationship between colliders and bodies let individual soap cells separate smoothly during simulation.

## AI Disclosure

I used Claude throughout the development of this project. It did not inform modeling choices and was primarily used to aid in Rust syntax, idioms, and tooling and with the Rapier and Salva APIs. 
