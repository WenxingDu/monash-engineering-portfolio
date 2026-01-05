# Discrete Time Feedback Control System Design

This project focuses on the design and analysis of a discrete-time feedback controller for a stochastic system, developed as part of ECE4132 (Control System Design).

The emphasis is not on implementing a standard controller structure, but on understanding how controller parameters affect stability, steady-state error, noise sensitivity, and predictability in a system with inherent randomness.


## Problem Overview

The system models a pricing mechanism for dispatching drones, where the controller adjusts the offered price based on the mismatch between demand and supply.

The key challenge is that the system output is noisy by nature:  
the number of drones dispatched is a random variable whose distribution depends on the current price through a nonlinear (logistic) function.

As a result, perfect error elimination is not possible, and controller design must explicitly account for noise and variability.


## Controller Structure

The controller is implemented as a discrete-time difference equation, where the price update depends on:
- the previous price,
- the current and previous error signals,
- and a small number of tunable parameters.

Rather than treating these parameters abstractly, their effects are analysed directly in the time domain.


## Parameter Analysis

Key controller parameters are examined in detail:

- **β (feedback accumulation)**  
  Analysed by interpreting the difference equation as an approximation to a continuous-time system.  
  Values of β greater or less than 1 are shown to introduce exponential growth or persistent steady-state error, respectively, while β = 1 leads to convergence centred around zero error.

- **κ (controller gain)**  
  Controls the sensitivity of the price update to error.  
  Larger values improve convergence speed but significantly amplify noise, while smaller values smooth the output at the cost of slower response.

An operating point is selected based on observed trade-offs between convergence, stability, and noise amplification rather than purely on speed.


## Noise and Predictability

Because the system output is stochastic, noise cannot be eliminated through control alone.

To assess predictability, the system is simulated repeatedly and statistical properties of the error and price signals are analysed.  
While individual runs remain noisy, averaging over multiple runs shows that the controller consistently converges to a stable operating price with small variance.

This highlights the distinction between single-run behaviour and expected system behaviour, which is critical for interpreting performance in stochastic control systems.


## Files

- `ECE4132 Project.py`  
  Python implementation of the controller and simulation.

- `ECE4132 Project Report.pdf`  
  Full derivations, plots, and detailed analysis of controller behaviour and parameter selection.


