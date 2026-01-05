# Intelligent Robotics - Integrated SLAM, Mapping and Navigation

This directory contains a full-stack mobile robotics project developed around
simultaneous localisation, mapping, and autonomous navigation in an unknown environment.

Beyond system integration, a large portion of the difficulty lies in the implementation and coordination of non-trivial algorithms under noisy, real-world conditions.  
Many components that are conceptually simple on paper required careful algorithmic design and repeated refinement to work reliably in practice.


## Problem Setting

The robot operates in an unknown arena containing:
- ArUco markers for localisation,
- target objects to be detected and visited in a given order,
- obstacle objects that must be avoided,
- and no prior map.

The system must estimate robot pose, build a consistent map, plan feasible paths, and execute them on a real robot, all while sensor measurements and motion are inherently noisy.


## Algorithmic and Implementation Challenges

Several parts of the project are algorithmically demanding and tightly coupled in code:

### SLAM and State Estimation
Pose estimation and landmark mapping are performed incrementally from noisy visual observations.  
Small errors in calibration, association, or state update can quickly accumulate, leading to map distortion or localisation failure.

Implementing this reliably required careful handling of:
- coordinate frame transformations,
- uncertainty propagation,
- landmark consistency over time,
- and rejection of low-confidence observations.

These issues are not visible in isolated tests, but emerge clearly in longer runs.


### Map Construction and Object Handling
Target objects and obstacles are detected separately from markers and must be integrated into a shared map representation.

This involves:
- managing multiple object types with different semantics,
- resolving duplicate detections,
- and updating estimated positions without corrupting the global map.

Designing this logic required balancing responsiveness with stability, rather than applying simple overwrite rules.


### Path Planning and Graph-Based Navigation
Navigation is built on a graph-based representation of the environment combined with sample-based planning.

The difficulty lies not in generating a path once, but in ensuring that:
- paths remain valid as the map evolves,
- clearance from obstacles is sufficient given localisation uncertainty,
- and planned trajectories are executable by the robot.

The code reflects repeated refinement of cost functions, waypoint spacing, and collision checks to avoid planner dead-ends and unsafe manoeuvres.


### Execution Logic and Failure Recovery
Even with a valid plan, execution frequently fails due to accumulated error or unexpected geometry.

The system therefore includes explicit logic for:
- detecting when execution is no longer safe,
- stopping or backing off near obstacles,
- triggering re-planning under degraded localisation,
- and adapting key parameters dynamically.

These behaviours required additional algorithmic structure rather than simple control flow.


## Design Decisions

Many algorithmic choices were guided by observed failure modes during full runs rather than theoretical optimality.

Examples include:
- preferring conservative landmark updates over aggressive map growth,
- trading path optimality for robustness and clearance,
- and introducing recovery logic as a core algorithmic component rather than an exception handler.

Fixes were rarely local: resolving one issue often required coordinated changes across estimation, mapping, and planning code.


## Notes on the Codebase

The codebase is intentionally kept explicit rather than minimal.  
Intermediate states, checks, and utility functions exist to make complex algorithmic interactions observable and debuggable.

The resulting structure reflects the reality of implementing robotics algorithms that must function together, not just pass individual unit tests.


## Final Remark

This project combines algorithmic complexity with real-world execution constraints.  
Its difficulty lies not only in implementing SLAM, mapping, or planning in isolation, but in making these algorithms interact correctly over long runs under uncertainty.
