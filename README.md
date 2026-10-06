# Flying Aces Pizza Delivery Simulator

A UAV delivery simulation for experimenting with autonomous drone fleet operations, mission planning, vehicle simulation, and delivery logistics in a real-world map environment.

The simulator models a fleet of delivery drones operating from a common base. Orders are generated over time or created interactively, grouped into missions, assigned to available drones, and flown through a simulated environment using [RotorPy](https://github.com/ethz-asl/rotorpy).

## Current Features

* Multi-drone fleet simulation
* Multiple drone configurations with named vehicle types
* Battery/energy simulation
* Automatic mission dispatch
* Batched delivery missions
* Delivery detection and mission completion
* Return-to-base behavior
* Drone recharging
* Mission abort/reset handling
* Procedural order spawning
* Interactive order creation by clicking the map
* Real-world map data loaded from JSON
* Building geometry used as flight obstacles
* Route visualization
* Interactive pan and zoom
* Wind visualization
* Per-drone telemetry and mission state in the HUD
* Scenario editing and saving/loading
* 3D drone dynamics and control through RotorPy

## Architecture

The simulator is divided into several major systems.

```text
Scenario
   │
   ├── Fleet
   ├── Map / Obstacles
   ├── Order Spawning
   ├── Weather / Events
   └── Configuration
          │
          ▼
     Mission Manager
          │
          ├── Mission Planning
          ├── Mission State
          ├── Fleet Assignment
          └── Delivery Management
                    │
                    ▼
                Drone
                    │
                    ├── Vehicle Dynamics
                    ├── Battery
                    └── Telemetry
                         │
                         ▼
                    RotorPy
```

The visualization layer operates independently of the core simulation logic:

```text
                 Simulation
                     │
        ┌────────────┼────────────┐
        ▼            ▼            ▼
      Map          Drones       Missions
        │            │            │
        └────────────┼────────────┘
                     ▼
                  Pygame
```

### Scenario

The scenario defines the environment in which the simulation runs. It brings together the map, fleet, orders, and simulation configuration.

The scenario system is intended to make different experiments reproducible without hard-coding the experiment directly into the UI.

### Map

`MapData` loads the map from JSON and provides the simulation with:

* Building geometry
* Roads and other map features
* Parks/green areas
* Water
* Delivery locations
* Obstacle geometry
* Map bounds

The map also handles the coordinate transformation between the source map representation and the simulator's coordinate system.

The renderer uses the same map data as the simulation, so the displayed environment and the environment used for obstacle-aware planning remain aligned.

### Camera

`Camera` handles the visualization coordinate system, including:

* Pan
* Zoom
* Screen/world coordinate conversion
* Fitting the map to the display

The camera is independent of the map and simulation state.

### Orders

Orders represent delivery requests.

Orders can currently be generated automatically through an `OrderSpawner` or created interactively by clicking on the map.

An order contains a delivery location and becomes eligible for mission planning when it enters the pending order queue.

### Mission Manager

`MissionManager` coordinates the fleet and manages the lifecycle of missions.

It is responsible for:

* Tracking available drones
* Receiving pending orders
* Grouping orders into delivery batches
* Planning missions
* Dispatching drones
* Tracking mission state
* Detecting deliveries
* Returning drones to base
* Managing recharge behavior
* Handling mission aborts and resets

The mission manager provides the main boundary between the high-level delivery system and individual drone simulation.

### Mission Planning

The mission planner determines how orders are grouped and how a drone should service them.

A mission may contain multiple deliveries, subject to the configured drone payload capacity and mission constraints.

Routes are represented in the simulation and rendered visually so that mission behavior can be inspected while the simulation is running.

### Drones

Each drone is represented by a vehicle configuration and runtime state.

Drone configurations define properties such as:

* Vehicle name
* Payload capacity
* Battery characteristics
* Vehicle dynamics parameters
* RotorPy configuration

Vehicle names are exposed to the simulation HUD. For example, multiple configurations can produce mission identities such as:

```text
Light-1
Endurance-1
Endurance-2
```

This makes it possible to distinguish vehicles in both telemetry and visualization.

### RotorPy

RotorPy provides the low-level multirotor simulation.

The simulator uses RotorPy for vehicle dynamics and control rather than implementing its own flight dynamics model.

The high-level simulation determines **what the drone should do**, while RotorPy handles **how the vehicle physically responds**.

### Telemetry

Telemetry is collected from the simulated drones and exposed to the visualization layer.

The HUD currently displays information including:

* Drone/mission identity
* Mission state
* Battery remaining
* Current delivery batch
* Pending orders

This allows the simulation to be observed without inspecting the underlying simulation state directly.

## User Interface

The simulator runs as an interactive Pygame application.

### Main Controls

| Control           | Function                                  |
| ----------------- | ----------------------------------------- |
| `A`               | Toggle automatic dispatch                 |
| `↑` / `↓`         | Increase/decrease order spawning interval |
| `SPACE`           | Pause/resume simulation                   |
| `P`               | Pause simulation                          |
| `S`               | Resume simulation                         |
| `R`               | Reset simulation                          |
| `ESC`             | Exit                                      |
| `N`               | Create a new/reset scenario state         |
| `TAB`             | Toggle simulation/editor mode             |
| `H`               | Toggle help                               |
| Middle mouse drag | Pan map                                   |
| Mouse wheel       | Zoom                                      |
| `Ctrl+S`          | Save scenario                             |
| `Ctrl+L`          | Load scenario                             |
| Left click        | Create a delivery order                   |

Some controls are mode-dependent.

## Map Data

Maps are stored as JSON files under `maps/`.

The current simulator uses:

```text
maps/richmond.json
```

Map coordinates are represented in a local projected coordinate system rather than latitude/longitude. This allows the simulation to work directly in metric world coordinates.

The map contains both visual features and geometry used by the simulation.

A typical map contains layers such as:

```text
buildings
roads
parks
water
delivery_locations
flight_corridors
no_fly_zones
```

Building geometry is particularly important because it provides obstacle information for flight planning.

## Running the Simulator

From the project directory:

```bash
python main.py
```

The simulator loads the configured scenario and map and opens the Pygame visualization.

The current default configuration includes a small multi-drone fleet, a delivery base, order spawning, and automatic dispatch.

## Standalone Map Generation

The project also contains tooling for generating map data from real-world geographic sources.

A map can be generated with:

```bash
python -m map_tools.map_builder --place "601 W Main St, Richmond, Virginia, USA" --diameter-miles 3 --output maps/richmond.json
```

The standalone map viewer can be launched with:

```bash
python map_viewer.py maps/richmond.json
```

The generated map is intended to be consumed by the simulator at runtime rather than converted into a pre-rendered image.

## Project Structure

The exact structure may evolve, but the major modules currently have these responsibilities:

```text
flyingacespizza/
│
├── pygame_ui.py          # Interactive Pygame application
├── scenario.py           # Scenario configuration/state
├── contracts.py          # Shared simulation data structures
├── mission_manager.py    # Fleet and mission lifecycle
├── mission_planner.py    # Delivery mission planning
├── drone.py              # Drone runtime behavior
├── drone_config.py       # Vehicle configurations
├── order_spawner.py      # Automatic order generation
├── editor.py             # Scenario editing
├── camera.py             # Map camera and transforms
├── map_module.py         # Map loading, geometry and rendering
│
├── maps/
│   └── richmond.json     # Current simulation map
│
└── map_tools/
    └── map_builder.py    # Geographic map generation
```

Additional modules support RotorPy integration, telemetry, vehicle control, and simulation-specific behavior.

## Simulation Loop

At a high level, each simulation update follows this pattern:

```text
1. Update scenario/events
        ↓
2. Generate new orders
        ↓
3. Update mission manager
        ↓
4. Plan/dispatch missions
        ↓
5. Update drone simulation
        ↓
6. Update mission and delivery state
        ↓
7. Update telemetry
        ↓
8. Render current state
```

The important distinction is that **mission logic and visualization are separate**. The Pygame UI observes and interacts with the simulation but is not responsible for implementing the underlying delivery logic.

## Current Scope

The simulator is currently focused on the operational behavior of a small UAV delivery fleet:

* Where should drones go?
* Which orders should be combined?
* When should a drone be dispatched?
* How does battery state affect a mission?
* What happens when a delivery is completed?
* How does the fleet recover and recharge?
* How does the simulated vehicle behave while executing the mission?

The simulator is **not currently an autonomous testing system**. Its purpose is to provide a controllable environment for experimenting with fleet behavior, mission planning, vehicle simulation, and visualization.

## Development Direction

The architecture is being developed toward a more general scenario-driven simulation in which the simulation itself can be run independently of the Pygame interface.

The intended separation is:

```text
Scenario
   │
   ├── Events / Weather
   ├── Order Spawning Strategies
   ├── Fleet
   ├── Map
   └── Configuration
          │
          ▼
      Simulation
          │
          ├── Mission Management
          ├── Mission Planning
          ├── Drone Simulation
          └── Telemetry
          
          │
          ▼
     Visualization
          │
          ├── Map
          ├── Drones
          ├── Routes
          ├── Orders
          └── HUD
```

This separation will allow the same simulation to eventually run headlessly for experiments, evaluation, and batch scenarios while retaining Pygame as an interactive visualization and debugging interface.
