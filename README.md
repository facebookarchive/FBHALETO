# General
FBHALETO (Facebook High Altitude Long Endurance Trajectory Optimization) toolbox was developed by Facebook to solve the problem of trajectory optimization for a solar powered High Altitude Platform Station Representative Air Vehicle (HAPSRAV).

## Project Overview
The trajectory optimization project considers the problem of a solar-powered UAV constrained within a geostationary cylinder (altitude and radius bounds), and how the vehicle should fly within these bounds to maximize net energy storage, or maximize the battery charge margin at night. The vehicle converts solar power into electricity, and is able to store power in a battery and as gravitational potential energy. The scenario of interest is winter solstice at a high latitude â€“ the day/location where the vehicle receives the least solar power. The challenge is to store as much energy as possible during the day so that the vehicle is able to survive the long night. Solar power collection is dictated by the local solar intensity and the angle between the sun and solar panels.

## References
Check out the Facebook research website, https://research.fb.com/category/connectivity/ .  The following conference papers were prepared with this toolbox:
- Bolandhemmat, H. et al, Energy Optimized Trajectory Planning for Solar Powered High Altitude Long Endurance Aircraft
- Marriott, J. et al, Trajectory Optimization of Solar-Powered High-Altitude Long Endurance Aircraft

## Requirements
This project was developed under Matlab/Simulink version 2017A.  It requires the following toolboxes:
* Aerospace Blockset
* Aerospace Toolbox
* Control Systems Toolbox
* Optimization Toolbox
* Simulink
* Stateflow
* Parallel Computing Toolbox (optional): Optimization can be very slow, so it is beneficial to install and run on a server with many cores, taking advantage of parallelization

## Description
- docs/ Contains a description of the aircraft model with associated systems; contains an explanation of high altitude winds and test stations used
- stations/ Test locations along +/- 35 degrees latitude
- steepest_descent/ Steepest descent method optimization files
- traj_opt/ Trajectory optimization models for simulation and optimization, main files:
  - EnergySim.m: Class which holds methods related to simulation initialization, parameterization data input, data output, data parsing, plotting, and comparisons
  - HAPSRAV_ENERGY_OPT.slx: Simulink model parameterized and called by optimization routine in EnergySim class
  - HAPSRAV_ENERGY_SIM.slx: Simulink model parameterized and called by simulation routine in EnergySim class
  - hapsrav_library.slx: Simulink library which holds the main blocks of the HAPSRAV_ENERGY_[OPT/SIM] models
  - get_rho.slx: Simulink model which calculates air density at altitude
  - atmospheric_library.slx: Simulink library containing atmospheric model used in get_rho.slx and HAPSRAV_ENERGY_[OPT/SIM]
  - panels.mat: MATLAB arrays holding panel normal directions and areas

## Examples
- traj_opt/example1.m: Simulation example with fig-8 orbit tracking solar azimuth angle.  manual winds = 1 m/sec
- traj_opt/example2.m: Simulation example with circular orbit.  autoWinds = true, 50th percentile winds
- traj_opt/example3.m: Successive hourlong optimization example with circular orbit baseline.  No winds
- traj_opt/example4.m: Simulation example with flight pattern.  No winds
- traj_opt/example5.m: Sample hourlong optimization example with fmincon, circular baseline, and no winds

## Join the FBHALETO community
See the CONTRIBUTING file for how to help out.

## License
By contributing to FBHALETO, you agree that your contributions will be licensed
under the LICENSE or COPYING file in the root directory of this source tree.
