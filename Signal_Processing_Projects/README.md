# Signal Processing Project – Gait Analysis from VGRF Signals

This project applies classical signal processing techniques to analyse human gait using vertical ground reaction force (VGRF) data measured from both feet.

The objective is to extract gait parameters such as stance time, swing time, and stride time from noisy force signals, using frequency-domain analysis and filtering rather than ad hoc thresholding.


## Project Focus

The work follows a structured signal processing pipeline:

- time-domain inspection of raw VGRF signals,
- frequency-domain analysis to identify dominant noise components,
- time–frequency analysis using windowed spectra,
- filter selection and design based on observed signal characteristics,
- event detection and gait parameter estimation on filtered signals.

Rather than assuming an appropriate filter a priori, the project uses spectral analysis to guide filtering decisions.


## Key Observations

Frequency-domain and spectrogram analysis show a strong noise component around 50 Hz, consistent with power-line interference.

Multiple filtering strategies were evaluated, including band-stop and low-pass filters.  
A low-pass FIR filter was selected as the most effective option for noise suppression while preserving gait-related dynamics.

After filtering, gait events (heel strike and toe-off) are detected using threshold-based logic on the cleaned signals, enabling reliable estimation of gait cycle parameters for both feet.


## Files

- `2111AssignmentGaitAnalysisCode/`  
  MATLAB code for signal analysis, filtering, and gait parameter extraction.

- `2111AssignmentReport.pdf`  
  Full report including spectral analysis, filter design rationale, and results.
