# Embedded Systems Projects

This folder contains two FPGA-based embedded systems projects completed as part of ECE2072 and ECE3073.

Both projects involve low-level hardware–software interaction on FPGA platforms, but differ significantly in scope:
ECE2072 focuses on synchronous digital design and FSM implementation, while ECE3073 extends to a full embedded system with a soft-core processor, RTOS, and hardware acceleration.

Detailed design explanations and results are documented in the accompanying project reports.


## ECE2072 – Digital System Design 

This project focuses on synchronous digital design using Verilog HDL on an FPGA board.

Key aspects include:
- modular Verilog design with clearly defined interfaces,
- timing-driven design based on a 50 MHz system clock,
- FSM implementation with explicit state encoding and transition logic,
- analysis of resource usage, including flip-flop count and FSM state complexity.

The design emphasises correctness, determinism, and clear reasoning about timing and state behaviour, rather than high-level abstraction.

Relevant files:
- `ECE2072_30171857.v` – Verilog implementation  
- `ECE2072_Report.pdf` – Design explanation and analysis  


## ECE3073 – FPGA-Based Embedded System with RTOS and Hardware Acceleration

This project is a substantially more complex embedded system built on a DE-10 FPGA board, combining
a soft-core Nios II processor, a real-time operating system, custom hardware accelerators, and a VGA display pipeline.

The system loads and processes grayscale images (160×120), supporting:
- full-screen and quad-image display modes,
- real-time image downscaling,
- image flipping,
- 3×3 blurring and Sobel edge detection.

### System-Level Challenges

The main difficulty of this project lies in coordinating **software, hardware, and timing constraints simultaneously**:

- A μC/OS-II RTOS is used to manage multiple concurrent tasks with carefully chosen priorities.
- Shared resources such as SDRAM, pixel buffers, and PIOs are protected using mutexes to avoid corruption and deadlocks.
- User inputs are handled through interrupts and semaphores rather than polling, to maintain responsiveness.

### Performance-Driven Design

Meeting real-time timing constraints required multiple iterations and trade-offs:

- Software-only implementations of convolution-based image processing were too slow.
- Reducing function call overhead in SDRAM access provided measurable speed improvements.
- Final performance requirements were met by designing a dedicated hardware convolution accelerator, which significantly reduced both computation and memory access overhead.

This accelerator exploits FPGA parallelism to offload 3×3 convolution operations from the CPU, enabling the system to exceed the required timing constraints.

### Why This Project Is Hard

This project goes beyond writing embedded C or Verilog in isolation.
It requires:
- understanding RTOS task scheduling and synchronisation,
- reasoning about memory bandwidth and access latency,
- designing hardware accelerators that integrate cleanly with software,
- and validating the system through benchmarking rather than single-run behaviour.

## Relevant files:
- `ECE3073_projectM2.qar` – Quartus project (hardware design)  
- `ECE3073_ProjectReport_Team103.pdf` – Full system design, benchmarking, and analysis  




