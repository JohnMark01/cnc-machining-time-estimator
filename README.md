# CNC Machining Time Estimator Using MATLAB

## Project Overview

An independently developed MATLAB-based CNC machining time estimation tool, created as my final-year BEng Mechatronic Engineering project at the University of Huddersfield.

The project focused on estimating machining cycle times directly from Siemens MPF CNC part programs, providing an alternative to conventional CAM-based time estimation methods.

## Project Objectives

- Develop a MATLAB program capable of interpreting CNC G-code and M-code instructions.
- Estimate machining times by analysing tool movements and machine operations.
- Account for machine-specific parameters, including spindle start-up delays, tool changes and rapid traverse speeds.
- Evaluate estimated cycle times against SolidWorks CAM predictions and experimental CNC machine timings.

## Methodology

The estimator processes CNC part programs by extracting relevant commands and tracking machine operating states.

It calculates time contributions from supported machining movements and auxiliary operations to produce an estimated total cycle time.

## Validation

The estimator was evaluated through comparisons with SolidWorks CAM timing predictions and recorded machining times from physical CNC experiments.

The project investigated differences between theoretical machining estimates and real machine operation, particularly the influence of machine-specific delays.

## Technologies

- MATLAB
- Siemens MPF / CNC G-code and M-code
- SolidWorks CAM
- CNC Machining
- Mathematical Modelling
- Experimental Validation

## Project Status

Completed undergraduate final-year project.

Original source code, experimental results and supporting documentation will be added as they are recovered and prepared for publication.

## Author

**Chinecherem John-Mark Obiajulu**

BEng (Hons) Mechatronic Engineering
