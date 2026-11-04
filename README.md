## Implementation

The CNC machining time estimator is implemented in MATLAB and processes Siemens MPF part programs to estimate machining cycle times.

The current implementation includes:

- **CNC program parsing:** Reads `.mpf`, `.nc` and `.txt` files.
- **Motion analysis:** Models rapid positioning, linear interpolation and circular interpolation.
- **Acceleration modelling:** Uses trapezoidal and triangular velocity profiles to estimate movement durations.
- **Machine operations:** Accounts for configurable tool-change, spindle start-up, coolant operation and dwell times.
- **Modal state tracking:** Tracks feed rates, coordinate positioning modes and selected machining planes.
- **Diagnostic logging:** Records certain unsupported CNC commands for further investigation.
- **Results reporting:** Generates estimated machining times and comparisons against configurable reference values.

## Development Status

**Working prototype — ongoing development**

This project originated as my final-year BEng Mechatronic Engineering dissertation at the University of Huddersfield.

The original academic implementation is functional and has been tested using Siemens MPF machining programs. The repository is being developed further as a personal engineering portfolio project.

### Planned Improvements

- Improve G-code and M-code recognition.
- Refine circular interpolation and complex motion calculations.
- Improve handling of machine-specific commands.
- Introduce individual experimental reference values for each machining program.
- Expand testing and validation using additional CNC programs.
- Improve documentation and code organisation.

### Validation Note

The estimator has been compared with experimental CNC machining timings and SolidWorks CAM predictions. Further validation and review of the reference datasets are planned before publishing final accuracy metrics.

## Running the Prototype

The MATLAB script reads supported CNC program files from its working directory or a configured ZIP archive.

1. Open `cnc_time_estimator.m` in MATLAB.
2. Review and configure the machine parameters.
3. Place the required CNC program files in the working directory.
4. Run the script to generate estimated machining times.

**Note:** The current version uses predefined reference timings for comparison. These values should be updated to match the individual test cases before interpreting the validation results.
