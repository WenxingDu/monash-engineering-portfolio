# Sound-Activated Lighting Circuit

This project is a complete analogue circuit design for a sound-activated lighting system, developed as part of ECE3161.

The circuit detects human footsteps using a microphone, filters and amplifies the signal, and drives an LED under low-light conditions.  
Rather than relying on digital processing, the design is fully analogue and built from discrete functional blocks.


## Design Objective

The system is intended for corridor lighting:

- During the day, the LED remains off regardless of sound.
- At night, detected footsteps trigger the LED.
- The light remains on for a short period after sound input, instead of flickering with transient noise.

This requires frequency-selective detection, sufficient gain, noise suppression, and controlled time delay.


## System Structure

The circuit is organised into several functional stages:

1. **Microphone Input and Bandpass Filtering**  
   A passive RC bandpass filter isolates frequencies between 100 Hz and 3 kHz, corresponding to typical footstep sounds, while rejecting low-frequency environmental noise and high-frequency interference.

2. **Non-inverting Operational Amplifier**  
   The filtered signal is amplified by a non-inverting op-amp with a gain of approximately 86, raising the microphone output from tens of millivolts to a level suitable for further processing.

3. **Half-wave Rectifier with Time Delay**  
   A diode-based rectifier and RC network convert the AC signal to DC while introducing a discharge delay of roughly 10 seconds.  
   This prevents rapid on–off switching of the LED due to intermittent sound.

4. **Voltage Comparator**  
   The rectified signal is compared against a reference voltage to produce a clean digital-level control signal once the sound amplitude exceeds a threshold.

5. **Light-Dependent Control and LED Driver**  
   A photoresistor-based circuit disables the LED during high ambient light conditions.  
   The LED is driven only when both sound and low-light conditions are satisfied.


## Design Considerations

Component values are selected based on analytical calculations and practical constraints such as available standard components.

Key considerations include:
- cutoff frequency placement and bandwidth of the bandpass filter,
- gain–noise trade-offs in the amplification stage,
- ripple reduction and discharge time in the rectifier,
- and reliable switching thresholds for the comparator and transistor stages.

The circuit behaviour is validated through LTspice simulations in both time and frequency domains.


## Power Consumption

Total power dissipation is approximately 25 mW, based on simulated current draw from ±5 V supplies, making the design suitable for low-power lighting applications.



## Files

- `3161ProjectDesign.asc`  
  LTspice schematic of the complete circuit.

- `ECE3161_ProjectReport_Wenxing_Du.pdf`  
  Full design rationale, calculations, and simulation results.

 
Detailed derivations and simulation plots are documented in the report.

