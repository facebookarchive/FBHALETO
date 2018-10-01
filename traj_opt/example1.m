% Copyright (C) 2018-present, Facebook, Inc.
%
% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; version 2 of the License.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License along
% with this program; if not, write to the Free Software Foundation, Inc.,
% 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
%

% EXAMPLE1.m
% MATLAB structs created: vehicle, time, init_conds, wind, par_params

clear;
d2r = pi / 180;
r2d = 180 / pi;

% Test mode and parameters
tst.autoWinds = false;  % Test winds (from NCAR)
% Replace these lines with some method that loads desired station,
% day of year, simulation start time, run time, vehicle lat/lon,
% wind levels, etc.
tst.location = '35N';
tst.wmag_level = 0.50;
tst.wdir_level = 0.99;
tst.wind_day = 'Winter'; % 'Winter', 'Fall', 'Summer', 'Spring'
tst.specDay = 180;  % Use specific day when tst.wind_day is ''

% wind - used when tst.autoWinds = false
wind.Wdir = 94.7195*pi/180;    % wind direction (rad)
wind.Wmag = 1.0;          % wind magnitude (m/s)
wind.Wd = 0.0;

% modes
md.wp_nav = true;       % Uses altitude hold and track hold
md.thrust_rev = false;   % Allow negative thrust

% time
t.year  = 2016;        % year of simulation start
t.month = 12;          % month of simulation start (1=jan to 12=dec)
t.day   = 21;          % day of simulation start (1 to [30,31,etc.])
t.hour  = 6;           % hour of simulation start (0=midnight to 23=11pm), not using DST
t.min   = 57;          % minute of simulation start
t.UTC   = -8;          % time zone (UTC offset)
t.run.hours = 5;      % simulation run hours
t.run.mins = 0;        % simulation run minutes

% init conds
ic.alt_init        = 18.29e3;       % initial altitude (m)
ic.chi_init        = 160 * d2r;     % initial course (rad)
ic.lat_init_deg    = 34.214729;     % initial latitude (deg)
ic.lon_init_deg    = -118.490845;   % initial longitude (deg)
ic.pN_init         = 1200;          % initial north position (m)
ic.pE_init         = 2000;          % initial east position (m)
ic.x_ctr           = 0.0;           % orbit center x-coord (m)
ic.y_ctr           = 0.0;           % orbit center y-coord (m)
ic.phi_0           = 0;             % initial bank angle (rad)
ic.chi0_rate_init  = 0;             % initial course rate (rad/s)
ic.alt0_rate_init  = 0;             % initial altitude rate (m/s)
ic.E_init          = 5;             % initial energy in battery (kWh)
ic.veas_init       = 7.26;          % initial equivalent airspeed (m/s)
ic.use_init_traj   = false;         % set true to use a provided initial trajectory from a previous
                                    % optimization run - note that this option is only useful if
                                    % the time durations would be identical; otherwise the previous
                                    % optimization results might not be necessarily applicable to the
                                    % next run - if this is true then the parametric sim params
                                    % block (couple lines below) doesn't have to be populated

% vehicle
v.configuration = 'AF_BASELINE';    % name of vehicle configuration - used for aero data lookup
v.alt_lim_l     = 18.29e3;          % minimum altitude for station-keeping (m)
v.alt_lim_h     = 24.574e3;         % maximum altitude for station-keeping (m)
v.w_xtrack      = 2 * pi * 0.03;    % frequency of crosstrack closed-loop (rad/s)
v.z_xtrack      = 0.7;              % damping coefficient for crosstrack
v.w_alt         = 2 * pi * 0.03;    % frequency of altitude closed-loop (rad/s)
v.z_alt         = 0.7;              % damping coefficient for altitude
v.tau_as        = 2 * pi * 0.03;    % closed-loop airspeed time constant (rad/s)
v.veas_rt_lim   = 0.03;             % equivalent airspeed rate limit (m/s^2)
v.phi_lim_alt_min = 0;              % low end for bank limit linear interpolation (m)
v.phi_lim_alt_max = 23e3;           % high end for bank limit linear interpolation (m)
v.phi_lim       = [5.0 10.0] * d2r; % bank angle limits at [phi_lim_alt_min, phi_lim_alt_max] (rad)
v.phi_rt_lim    = [1.0 3.0] * d2r;  % bank rate limits (rad/s)
v.gamma_lim     = -3.0 * d2r;       % flight path angle limit (rad)
v.S             = 136.75;           % wing planform area (m^2)
v.b             = 74.953;           % wingspan (m)
v.M             = 571.26;           % vehicle mass, including battery (kg)
load('panels.mat');                 % vectors of panel element areas and directions (in vehicle body axes)
v.panel_dirs    = panel_dirs;       % array of panel unit normal vectors in body axes (#panels by 3)
v.panel_areas   = panel_areas;      % vector of panel areas, in same order as panel_dirs
v.panel_eff     = 0.23;             % panel efficiency
v.mppt_eff      = 0.97;             % maximum power-point tracking efficiency
v.P_acc         = 360;              % "accessory" power draw - avionics, payload, flight computers, heaters, etc. (W)
v.R_prop        = 1.5;              % propeller radius (m)
v.num_motors    = 4;                % number of motors/propellers
v.prop_eff_scale= 0.90;             % scale max. theoretical propeller efficiency by this constant
v.motor_eff     = 0.95*0.97;        % motor+motor controller efficiency
v.charge_eff    = 0.93;             % battery charge efficiency
v.discharge_eff = 0.97;             % battery discharge efficiency
v.E_batt_max    = 0.320*200;        % max battery energy (kWh) - specific density (kWh/kg) times mass (kg)

% parametric sim params (used for baseline solution)
%   1: Circular orbit (ad hoc)
%   2: Racetrack orbit tracking sun azimuth angle
%   3: Figure eight orbit tracking sun azimuth angle
%   4: Circular orbit (vector fields, preferred with wind)
%   5: Flight plan pattern (arbitrary trajectory)
pp.chi_cmd_mode        = 3;         % course command mode
pp.orbit_dir           = 0;         % orbit direction: 1 - CCW, 0 - CW
pp.orbit_rho           = 3000;      % orbit radius (m) (racetrack major axis)
pp.orbit_start         = true;      % start AV position and course at orbit
pp.min_axis            = 1500;      % racetrack minor axis (m)
pp.turn_time_circle    = 60*5.5;    % mode 1: time to turn 180 deg (s)
pp.turn_time_rt        = 60*2;      % mode 2: time to turn 180 deg (s)
pp.hold_time           = 60*3.5;    % mode 2: time to hold straight portion (s)
pp.rt_azimuth_offset   = 0;         % mode 2: angle between straight portion and sun azimuth (rad)
pp.fig_eight_time      = 60*10;     % mode 3: time to complete one period of figure-eight (s)
                                    %         note: figure-eight parameterized for wind is only working with 10-min period
pp.f8_azimuth_offset   = pi/2;      % mode 3: angle between major axis of figure-eight and sun-azimuth (rad)
pp.alt_cmd_mode        = 1;         % altitude command mode
%   1: (Excess) power-limited climb
%   2: Periodic climb/glide (combine with racetrack)
pp.alt_rt_lim_periodic = 0.58;      % mode 2: climb/descent rate for periodic climb/glide (m/s)
pp.veas_night_mode     = 7.26;      % equivalent airspeed when sun is below horizon

%   Enable Fuzzy Logic Network
pp.enableFuzzyLogic   = 0 ;         % Enables the Fuzzy logic network for the energy optimised trajectory planning - md.wp_navmust be true

% optimization params: this struct is only necessary for optimizations
op.n      = 6;                      % num. of time intervals to discretize simulation into - 60 to 180 per hour is a good baseline
% lower bounds: alt rate (m/s), Veas (m/s), bank angle (rad)
op.lb     = [-2.08 * ones(1,op.n), 7.1 * ones(1,op.n), -0.1745 * ones(1,op.n)];
% upper bounds: alt rate (m/s), Veas (m/s), bank angle (rad)
op.ub     = [ 2.08 * ones(1,op.n), 7.6 * ones(1,op.n),  0.1745 * ones(1,op.n)];
op.pll    = false;                  % use parallel computing toolbox?
op.fevals = 400*op.n;               % maximum number of function evaluations - 1000*n is a good baseline

% ---- EXAMPLE ----
% create an EnergySim instance parameters above
ES = EnergySim(md,v,t,ic,wind,pp,op,tst);

% run the optimization using the parameters that ES was instantiated with
ES.runSim();

% plot data from baseline(0)
ES.plotSim(0);

% save ES object to a .mat file
file_str = ['ES_test_', num2str(posixtime(datetime('now')),10),'.mat'];
save(file_str, 'ES');
