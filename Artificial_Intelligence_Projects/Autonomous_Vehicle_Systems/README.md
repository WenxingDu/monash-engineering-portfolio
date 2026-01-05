## Project Structure

The repository consists of two closely connected modules:

### 1. Planning, Trajectory Design and Receding Horizon Control  
(Assignment 1)

This module focuses on generating collision-free and dynamically feasible vehicle motion in a structured parking-lot environment.

Key aspects include:
- Binary occupancy grid construction with obstacle inflation
- Global path planning using RRT* and Hybrid A*
- Systematic parameter analysis of Hybrid A* (motion primitives, expansion interval, primitive length)
- Trajectory smoothing using geometric arc interpolation to remove kinematically infeasible sharp turns
- Arc-length–parameterized cubic spline paths
- Trapezoidal and triangular velocity profiling under acceleration and speed constraints
- Receding Horizon Planning (RHP) for trajectory tracking with nonlinear constraints

The design deliberately separates global feasibility from local executability, allowing the controller to focus on optimization and robustness instead of compensating for unrealistic references.

Relevant files:
- CIV4100_Assignment1_Code_Du/
- CIV4100_Assignment_1_Report_Du.pdf


### 2. Perception, Robustness Testing and Adversarial Defence  
(Assignment 2)

This module investigates the perception side of autonomous driving, with a focus on model reliability rather than raw accuracy.

A traffic sign classification system was developed using transfer learning (AlexNet) on a highly imbalanced dataset that was manually analysed, restructured, and balanced through staged augmentation. While several modern CNNs achieved high validation accuracy, their performance degraded significantly on the Kaggle test set, highlighting the gap between validation metrics and real generalisation.

Key components include:
- Dataset inspection and restructuring (class imbalance, resolution variability)
- Controlled class balancing and augmentation strategy
- Transfer learning with a lightweight CNN architecture
- Careful label mapping and submission consistency for real-world evaluation
- Metamorphic testing under blur, brightness, rotation, and horizontal flip
- Simulated non-gradient adversarial perturbations
- Defence strategies via input denoising and adversarial-style retraining

Rather than assuming invariance is always desirable, the analysis shows that certain sensitivities (e.g. to horizontal flipping) may reflect meaningful spatial semantics in traffic signs, not necessarily model failure.

Defence experiments further demonstrate that simple preprocessing (Gaussian blur) can outperform retraining in non-gradient attack scenarios, offering a practical trade-off between robustness and system complexity.

Relevant files:
- CIV4100_Assignment2_Code_Du/
- CIV4100_Assignment_2_Report_Du.pdf



## Design Perspective

Across both modules, the following principles guided the system design:

- High validation accuracy does not guarantee reliable deployment
- Planning and control should not compensate for unrealistic upstream assumptions
- Perception models must be tested beyond clean data
- Simple, interpretable defences can be more effective than complex retraining
- Trade-offs between performance, robustness, and computation must be explicit

Overall, this repository reflects a system-level view of autonomous vehicles, combining planning, control, perception, and robustness considerations rather than optimising each component in isolation.

