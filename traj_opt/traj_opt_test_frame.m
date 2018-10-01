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

% This script serves the purpose of performance evaluation of our
% optimization routines.


clear;
d2r = pi / 180;
r2d = 180 / pi;

% ======== Begin specify test case info ========

% sun rise or sunset
% 0 --> sun rise
% 1 --> daytime energy collecting (middle of the day)
% 2 --> afternoon climbing (one hour after mid day)
% 3 --> sunset
sun_time = 0;

% minutes before/after sun_time as indicated above
sun_time_before = 0;
sun_time_after = 60;

% Wind test mode and parameters
tst.autoWinds = false;  % Test winds (from NCAR)
% Replace these lines with some method that loads desired station,
% day of year, simulation start time, run time, vehicle lat/lon,
% wind levels, etc.
tst.location = '30N';
% wind percentile setting for magnitude and direction
% 25%, 50%, 68.27%, 75%, 90%, 95%, 99%
tst.wmag_level = 0.25;
tst.wdir_level = 0.99;
tst.wind_day = 'Winter'; % 'Winter', 'Fall', 'Summer', 'Spring'
tst.specDay = 180;  % Use specific day when tst.wind_day is ''

load(fullfile(pwd, '../stations', strcat('sttn_deg', tst.location, '.mat')));

% get the season of the current test case
day_idx = setTestDay(tst, tst_station);
season_idx = find(tst_station.dayIdx == day_idx);
if isempty(season_idx)
    error('Invalid season provided for this test run.')
end

% init conditions based on different test time
if sun_time == 0 || sun_time == 1
    ic.alt_init        = 18.29e3;
    ic.E_init          = 5;
elseif sun_time == 2
    ic.alt_init        = 18.29e3;
    ic.E_init          = 64;
elseif sun_time == 3
    ic.alt_init        = 24.450e3;  % lower than highest altitude, descent starts earlier than 1 hour before sunset
    ic.E_init          = 64;
else
    error('sun_time input is invalid.')
end

% wind - used when tst.autoWinds = false
wind.Wdir = genTestWindDir(tst_station, tst, ic.alt_init);   % wind direction (rad)
wind.Wmag =  0.0;                                            % wind magnitude (m/s)
wind.Wd = 0.0;
% ======== End specify test case info ========


% Display test information of this run
disp(['This is a test case at ', tst_station.name, ', lat = ', num2str(tst_station.lat), ', lon = ', num2str(tst_station.lon)]);
if sun_time == 0
    sun_time_str = 'Sunrise';
elseif sun_time == 1
    sun_time_str = 'Middle of day';
elseif sun_time == 2
    sun_time_str = 'Afternoon climbing';
elseif sun_time == 3
    sun_time_str = 'Sunset';
else
    error('Invalid sun_time');
end
disp(['Season is ', tst.wind_day, '; ', sun_time_str, '; duration = [-', num2str(sun_time_before), ', ', num2str(sun_time_after), '] min']);
disp(['Initial battery SOC = ', num2str(ic.E_init ), 'kWh; Initial altitude = ', num2str(ic.alt_init), 'm']);
if tst.autoWinds == true
    disp(['Autowind test is on, using ', num2str(tst.wmag_level), ' percentile magnitude.']);
else
    if wind.Wmag > 0
        disp(['Autowind test is off, ', 'constant wind test is on. ', 'Wind magnitude = ', num2str(wind.Wmag),...
              ' m/s, Wind direction = ', num2str(wind.Wdir * r2d), ' degree.']);
    else
        disp('No wind test case.');
    end
end

% modes
md.wp_nav = false;      % Uses altitude hold and track hold for the baseline trajectory planning - setting it to true will result in using the altitude rate and coarse rate commands from the optimization routine
md.thrust_rev = false;   % Allow negative thrust


% time
[t0, t1] = parse_time(tst_station, season_idx, sun_time, sun_time_before, sun_time_after);

ti.year  = t0.Year;        % year of simulation start
ti.month = t0.Month;          % month of simulation start (1=jan to 12=dec)
ti.day   = t0.Day;         % day of simulation start (1 to [30,31,etc.])
ti.hour  = t0.Hour;           % hour of simulation start (0=midnight to 23=11pm), not using DST
ti.min   = t0.Minute;           % minute of simulation start

tf.year  = t1.Year;       % year of simulation end
tf.month = t1.Month;         % month of simulation start (1=jan to 12=dec)
tf.day   = t1.Day;          % day of simulation start (1 to [30,31,etc.])
tf.hour  = t1.Hour;         % hour of simulation start (0=midnight to 23=11pm), not using DST
tf.min   = t1.Minute;         % minute of simulation start

t.UTC        = 0;      % time zone (UTC offset)
t.time_init  = ti;
t.time_fin   = tf;


% ic.alt_init        = 24.574e3;       % initial altitude (m)
ic.chi_init        = 160 * d2r;     % initial course (rad)
ic.lat_init_deg    = tst_station.lat;     % initial latitude (deg)
ic.lon_init_deg    = tst_station.lon;   % initial longitude (deg)
ic.pN_init         = 1200;          % initial north position (m)
ic.pE_init         = 2000;          % initial east position (m)
ic.x_ctr           = 150.0;           % orbit center x-coord (m)
ic.y_ctr           = 220.0;           % orbit center y-coord (m)
ic.phi_0           = 0;             % initial bank angle (rad)
ic.chi0_rate_init  = 0;             % initial course rate (rad/s)
ic.alt0_rate_init  = 0;             % initial altitude rate (m/s)
% ic.E_init          = 5;             % initial energy in battery (kWh) - 64 is the max
ic.veas_init       = 7.26;          % initial equivalent airspeed (m/s)
ic.use_init_traj   = false;         % set true to use a provided initial trajectory from a previous optimization run - note that this option is only useful if
                                    % the time durations would be
                                    % identical; otherwise the previous
                                    % optimization results might not be
                                    % necessarily applicable to the next
                                    % run - if this is true then the parametric
                                    % sim params block (couple lines below) doesn't have to be populated.
                                    %
ic.init_traj       = '';            % if use_init_traj flag is true, then the provided optimized trajectory data is used as the initial trajectory
                                    % the data must be included in the matlab path directory


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
pp.chi_cmd_mode        = 4;         % course command mode
pp.orbit_dir           = 0;	        % orbit direction: 1 - CCW, 0 - CW
pp.orbit_rho           = 2010;      % orbit radius (m)
pp.epsln_circ          = 40*pi/180; % width of boundary region around sliding mode (rad)
pp.k_c                 = 5;         % rate of transtion for 90 deg turn
pp.K_circ              = 453.6;     % sliding mode region gain
pp.alpha_circ          = pp.K_circ / 1.2;  % Course dynamics time constant
pp.turn_time_circle    = 60*4.5;    % mode 1: time to turn 180 deg (s)
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


% optimization params
op.n       = 180;                  % num. of time intervals to discretize simulation into - 60 to 180 per hour is a good baseline
op.optVeas = true;                 % set to true to include airspeed as an unknown in the optimization process

if (op.optVeas == true )
    % lower bounds: alt rate (m/s), Veas (m/s), bank angle (rad)
    op.lb     = [-2.08 * ones(1,op.n), 7.1 * ones(1,op.n), -0.1745 * ones(1,op.n)];
    % upper bounds: alt rate (m/s), Veas (m/s), bank angle (rad)
    op.ub     = [ 2.08 * ones(1,op.n), 7.6 * ones(1,op.n),  0.1745 * ones(1,op.n)];
else
    % lower bounds: alt rate (m/s), bank angle (rad)
    op.lb     = [-2.08 * ones(1,op.n), -0.1745 * ones(1,op.n)];
    % upper bounds: alt rate (m/s), bank angle (rad)
    op.ub     = [ 2.08 * ones(1,op.n),  0.1745 * ones(1,op.n)];
end

op.pll     = true;                   % use parallel computing toolbox?
op.fevals  = 1000 * op.n;                % maximum number of function evaluations - 1000*n is a good baseline

%Choices for the optimisation routine
% 0 --> fmincon
% 1 --> simplex search algorithm
% 2 --> Fourier Transformation, NEED to specify op.fft_order
% 3 --> genetic algorithm
% 4 --> custom gradient steepest descent with projection
op.opt_opt  = 0 ;

% set number of coefficients for Fourier Transformation;
% number of optimization variables will be
% 2*sum(N_fft_order)+3;
% [8,8,15] is the minimum number to produce comparable
% results with time domain-variables when running 1-hour optimization;
% For complete study of N_fft_order, refer to the
% presentation: https://fburl.com/8tsg54wy
op.fft_order = [0, 0, 0];

%Choices for the cost function
% 0 --> first order cost function from t0 to tfin
% 1 --> signed sum of the second order energy cost functions for each period
% 2 --> minimization of the angle difference between the sun and aircraft normal vectors
op.opt_cost = 0 ;

% Altitude error weighting in the energy cost function - either set it to 1
% 1      -->  is default - basically unity weighting
% 0.01   -->  encourage the optimization routine to store more energy in
% form of potential energy once the batteries are fully charged.
op.alt_weight = 1 ;

% ---- EXAMPLE ----
% create an EnergySim instance parameters above
ES = EnergySim(md,v,t,ic,wind,pp,op,tst);

% run the optimization using the parameters that ES was instantiated with
ES.runOpt();

% compare the energy storage difference between the baseline and optimal
ES.compareSimAndOpt();

% plot data from optimal (1) or baseline(0)
ES.plotSim(1);


% save ES object to a .mat file
file_str = ['ES_test_', num2str(posixtime(datetime('now')),10),'.mat'];
save(file_str, 'ES');



%% ===== helper functions ====

function [t0, tf] = parse_time(tst_station, season_idx, sun_time, time_before, time_after)

    if sun_time == 0       % sunrsie
        sun_t = tst_station.sunrise(season_idx);
    elseif sun_time == 1   % middle of the day
        sun_t = tst_station.sunrise(season_idx) + 0.5 * tst_station.dayLength(season_idx);
    elseif sun_time == 2   % one hour after mid day
        sun_t = tst_station.sunrise(season_idx) + 0.5 * tst_station.dayLength(season_idx) + 1;
    elseif sun_time == 3   % sunset
        sun_t = tst_station.sunset(season_idx);
    else
        error('Invaid sun_time parameter, the value has to be 0 or 1.')
    end

    month       = str2double(datestr(doy2date(tst_station.dayIdx(season_idx), 2017), 'mm'));
    day         = str2double(datestr(doy2date(tst_station.dayIdx(season_idx), 2017), 'dd'));

    start_t     = sun_t - time_before / 60;
    start_t_h   = fix(start_t);
    start_t_min = round(rem(start_t, 1) * 60);
    t0          = datetime(2017, month, day, start_t_h, start_t_min, 0);


    end_t       = sun_t + time_after / 60;
    end_t_h     = fix(end_t);
    end_t_min   = round(rem(end_t, 1) * 60);
    tf          = datetime(2017, month, day, end_t_h, end_t_min, 0);

end
