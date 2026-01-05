# Statistics and Data Analysis Projects

This folder contains coursework-based projects focused on probability modelling, statistical inference, and data-driven analysis using MATLAB.

The work emphasises reasoning from data, rather than treating statistics as a set of formulas.  
All analysis is supported by explicit mathematical definitions and verified through computation.


## Public Transport Data Analysis (ECE2191)

This project analyses real-world public transport survey data provided by a local government transport department.

The dataset includes household-level and trip-level information, which is used to study travel behaviour from a probabilistic perspective.

Key aspects of the work include:

- Data filtering and preprocessing in MATLAB  
  (e.g. isolating public transport trips and handling household/person indexing)
- Modelling continuous random variables such as trip distance and passenger age
- Distribution fitting and comparison (uniform, Gaussian, exponential, gamma)
- Joint, marginal, and conditional probability mass functions (PMFs)
- Statistical dependence testing between discrete random variables
- Sample mean behaviour, variance analysis, and interpretation using the law of large numbers

Rather than assuming a model upfront, distributions are selected by comparing empirical data with fitted probability models and interpreting the results.


### What This Project Focuses On

- Translating mathematical probability definitions into working code  
- Understanding *when* common assumptions (e.g. independence, linear correlation) fail
- Interpreting statistical results in a real data context, not just computing them
- Connecting theoretical results (sample mean, variance, Chebyshev inequality, law of large numbers) to observed behaviour in finite datasets

The emphasis is on understanding statistical behaviour, not on optimising metrics or producing polished dashboards.


## Files

- `2191Assignment2Code/`  
  MATLAB scripts and live scripts used for data processing, probability modelling, and visualisation.

- `2191Assignment2Report_compressed.pdf`  
  Full report containing detailed derivations, figures, code explanations, and interpretation of results.

