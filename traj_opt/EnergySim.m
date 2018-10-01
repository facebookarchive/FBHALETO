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
classdef EnergySim < handle
    %   Synopsis: ENERGYSIM Deals with input, output, and execution of Simulink models for simulation or optimization.
    %   Inherits handle class making objects of this class pointers


    properties (SetAccess = private)

        % the following are all MATLAB structs
        vehicle         % holds vehicle-related parameters
        init_conds      % holds initial conditions for simulations
        testStation     % holds test station with wind conditions
        testOptions     % holds test options
        time            % holds data on start and end dates/times
        wind            % holds wind data
        sim_out         % holds Simulink SimOut object for a simulation
        sim_out_parsed  % holds parsed simulation data
        opt_data_out    % holds optimization output (i.e. f_opt, X_opt)
        opt_cmd         % holds commands associated with optimal solution
        opt_out         % holds Simulink SimOut object for optimal soln
        opt_out_parsed  % holds parsed simulation data for optimal
        wp_nav          % true - WP navigation, false - trajectory optimization
        thrust_rev      % true - neg thrust, false - no neg thrust

    end

    properties (SetAccess = immutable)

        dT  = 0.5              % base simulation time step
        dT_fixed = 0.5         % copy of simulation time step - hack for autocoding
        dT_trig = 30           % simulation time step for triggered subsystems (solar vector/intensity updates, etc.)
        aero                   % struct that holds aero data - it is set during initialization
        absorb_h               % for atmospheric absorption calculation
        alpha_h                % for atmospheric absorption calculation
        sys_diff               % discrete time differentiation
        sys_lpf                % discrete time low pass filtering
        par_params             % parameters for parametric simulations
        opt_params             % parameters for optimization

    end

    properties (Constant)

        g   = 9.80665          % earth gravitational constant
        d2r = pi / 180
        r2d = 180 / pi
        earth_rho = 6378137.0  % earth radius
        rho_SL  = 1.225        % kg/m^3, at sea level
        Gsolar = 1367          % solar constant (W/m^2)

    end

    methods
        function obj = EnergySim(md, vehicle, time, init_conds, wind, par_params, opt_params, tst)
            % obj = EnergySim(md, vehicle, time, init_conds, wind, par_params, opt_params, tst)
            %   Create an EnergySim object parametrized by inputs,
            %   which can be set in a script

            % do one-time calculations for atmospheric absorption
            [obj.absorb_h, obj.alpha_h] = EnergySim.atmosAbsorb();

            % fill in run-time options
            obj.wp_nav = md.wp_nav;
            obj.thrust_rev = md.thrust_rev;

            % fill in test options
            obj.testOptions = tst;

            % assign the test station struct
            %   This must always be assigned, even if
            %   autoWinds is not selected
            obj.testStation = load(fullfile(pwd,'../stations',strcat('sttn_deg',obj.testOptions.location,'.mat')));

            % fill in the vehicle struct
            obj.vehicle = EnergySim.setVehicle(vehicle);

            % fill in the initial conditions struct
            obj.init_conds = EnergySim.setICs(init_conds);

            % fill in the wind struct
            if (obj.testOptions.autoWinds)
                obj.testOptions.dayIdx = setTestDay(obj.testOptions, obj.testStation.tst_station);
                obj.wind = EnergySim.testWind(wind, obj.testStation.tst_station, obj.testOptions, init_conds.alt_init);
            else
                obj.testOptions.dayIdx = 183;
                obj.wind = EnergySim.setWind(wind);
            end

            % fill in the time struct
            obj.time = EnergySim.setTime(time);

            % fill in the parametric simulation struct
            obj.par_params = EnergySim.setParParams(par_params);

            % fill in the optimization parameter struct
            if (~exist('opt_params','var'))
                disp('No optimization parameters supplied - optimization will not be possible');
            else
                obj.opt_params = EnergySim.setOptParams(opt_params);
            end

            % fill in the aero data struct
            obj.aero = EnergySim.setAeroData(vehicle);

            % set the discrete-time differentiation
            obj.sys_diff = obj.tfToSS(0.2, 1);

            % set the discrete-time low pass filtering
            obj.sys_lpf  = obj.tfToSS(1.0, 0);

        end

        function runSim(obj)
            % runSim(obj)
            %   Prepare, run, and parse a single simulation

            % set the opt_flag - this is not an optimization (only a sim)
            opt_flag = false;

            obj.wp_nav = true ;     % set it true to use the baseline sim parameters

            % trim aircraft with inial conditions
            obj.trimAC();

            % create a new instance of a Simulink.SimulationInput class
            % with the Simulink model used for single simulations
            sim_in = Simulink.SimulationInput('HAPSRAV_ENERGY_SIM');

            % import required simulation parameters to sim_in
            si = obj.setSimOrOpt(sim_in, opt_flag);

            disp(['Simulation of ',num2str(obj.time.tfin/(60*60)),' hours starting ',datestr(datetime(obj.time.t_init, 'ConvertFrom','posixtime')),' has begun.'])
            start_timer = tic;

            % run the simulation
            obj.sim_out = sim(si);

            stop_timer = toc(start_timer);
            disp(['Simulation completed in ',num2str(stop_timer),' seconds.'])

            % take SimOut object and extract all the signals
            obj.sim_out_parsed = obj.parseSim(obj.sim_out, opt_flag);

        end

        function runSimGivenX(obj, X)
            % given a series of cmds X, run sim and store the output as
            % optimization results

            obj.runSim();

            obj.opt_data_out.X = X;

            opt_in = Simulink.SimulationInput('HAPSRAV_ENERGY_OPT');

            %set the wp_nav flag false now
            obj.wp_nav = false ;     % use the optimized commands for the sim

            % import required simulation parameters to sim_in
            oi = obj.setSimOrOpt(opt_in, false);


            % run sim with the commands for the optimal solution in order
            % to extract data from the SimOut object
            obj.opt_out = obj.runSimWithOpt(oi);

            % turn off fast restart, so MATLAB can close model without
            % prompt which causes problems when running in background
            if bdIsLoaded('HAPSRAV_ENERGY_OPT')
                set_param('HAPSRAV_ENERGY_OPT','FastRestart','off');
            end

            % take SimOut object and extract all the signals
            obj.opt_out_parsed = obj.parseSim(obj.opt_out, true);
        end

        function runOpt(obj)
            % runOpt(obj)
            %   Prepare, run optimizer, and parse data from optimal
            %   solution

            % for shorthand - number of time intervals
            n = obj.opt_params.n;

            % set opt_flag - this is an optimization
            opt_flag = true;

            % run a simulation to determine bank angle commands for
            % "baseline" to initialize optimizer with (this is mostly
            % needed for wind)
            opt_init = obj.initBankOpt(n);

            % set the intiial values of optimization variables
            if (obj.opt_params.optVeas == true)
                X0 = [opt_init.dAlt_init.Data(2:end)', obj.init_conds.veas_init * ones(1,n), opt_init.bank_init.Data(2:end)'];
            else
                X0 = [opt_init.dAlt_init.Data(2:end)' , opt_init.bank_init.Data(2:end)'];
            end


            % load the system here
            if ~bdIsLoaded('HAPSRAV_ENERGY_OPT')
                load_system('HAPSRAV_ENERGY_OPT');
            end

            % create a new instance of a Simulink.SimulationInput class
            opt_in = Simulink.SimulationInput('HAPSRAV_ENERGY_OPT');

            %set the wp_nav flag false now
            obj.wp_nav = false ;     % use the optimized commands for the sim

            % import required simulation parameters to sim_in
            oi = obj.setSimOrOpt(opt_in, opt_flag);

            % turn on fast restart (for non-parallel jobs at least)
            set_param('HAPSRAV_ENERGY_OPT','FastRestart','on');

            % if parallel computing is used for optimization, this gets
            % parallel computing set up and fixes a bug where MATLAB
            % crashes otherwise
            if obj.opt_params.pll
                disp('Initializing parallel computing.')

                t_evals      = linspace(0, obj.time.tfin, n+1);
                alt_rt_evals = [obj.init_conds.alt0_rate_init, X0(1:n)      ];

                if(obj.opt_params.optVeas == true)
                    veas_evals   = [obj.init_conds.veas_init,      X0(n+1:2*n)  ];
                    phi_evals    = [obj.init_conds.phi_0,          X0(2*n+1:end)];
                else
                    veas_evals   = [obj.init_conds.veas_init,      obj.init_conds.veas_init*ones(1,n)  ];
                    phi_evals    = [obj.init_conds.phi_0,          X0(n+1:end)];
                end



                alt_rt_cmd = timeseries(alt_rt_evals, t_evals);
                eas_cmd    = timeseries(veas_evals,   t_evals);
                phi_cmd    = timeseries(phi_evals,    t_evals);

                oi_tmp = obj.setCmds(oi, alt_rt_cmd, eas_cmd, phi_cmd);

                % runs a single simulation with parallel computing to make
                % MATLAB happy
                parsim(oi_tmp,'TransferBaseWorkspaceVariables',false,'UseFastRestart',true);
            end

            disp('Optimization has begun.')
            start_timer = tic;

            % run the optimization routine
            obj.opt_data_out = obj.optRoutine(X0, oi);

            end_timer = toc(start_timer);
            disp(['Optimization completed in ', num2str(end_timer), ' seconds with exit flag ', num2str(obj.opt_data_out.exitflag),'.']);

            % run sim with the commands for the optimal solution in order
            % to extract data from the SimOut object
            obj.opt_out = obj.runSimWithOpt(oi);

            % turn off fast restart, so MATLAB can close model without
            % prompt which causes problems when running in background
            if bdIsLoaded('HAPSRAV_ENERGY_OPT')
                set_param('HAPSRAV_ENERGY_OPT','FastRestart','off');
            end

            % take SimOut object and extract all the signals
            obj.opt_out_parsed = obj.parseSim(obj.opt_out, opt_flag);

        end


        function plotSim(obj, opt_flag)
            % plotSim(obj, opt_flag)
            %   Create various figures with the output of the simulation or
            %   optimization. "opt_flag" is a boolean specifying whether
            %   data from the optimizer or baseline is to be output.

            % make sure the struct to plot data from exists
            if ~opt_flag
                so = obj.sim_out_parsed;
                if(isempty(so))
                    error('Error: It appears that simulation has not been run/parsed yet');
                end
            else
                so = obj.opt_out_parsed;
                if(isempty(so))
                    error('Error: It appears that optimization has not been run/parsed yet');
                end
            end

            % sanity check
            if(so.t(end) ~= obj.time.tfin)
                error(['Error: Time from simulation output (',num2str(so.t(end)),') does not match desired sim time (', num2str(obj.time.tfin),')']);
            end

            % shorthand - length of the simulated time period in seconds
            tfin = obj.time.tfin;

            % plot winds data
            figure;
            subplot(311)
            plot(so.t_hr, so.wnd_n)
            grid on;
            title('Winds')
            xlabel('t (h)')
            ylabel('Wnd North (m/s)')
            subplot(312)
            plot(so.t_hr, so.wnd_e)
            grid on;
            xlabel('t (h)')
            ylabel('Wnd East (m/s)')
            subplot(313)
            plot(so.t_hr, so.wnd_d)
            grid on;
            xlabel('t (h)')
            ylabel('Wnd Down (m/s)')
            xlim([0 tfin/(60*60)])

            % plot solar data
            figure('Position',[100,100,960,640]);
            subplot(2,2,1)
            plot(so.t_hr, so.P_el_in)
            grid on;
            title('Solar Power Collection')
            xlabel('t (h)')
            ylabel('Power (kW)')
            xlim([0 tfin/(60*60)])

            subplot(2,2,2)
            plot(so.t_hr, so.E_el_in)
            grid on;
            title('Solar Energy Collection')
            xlabel('t (h)')
            ylabel('Energy (kWh)')
            xlim([0 tfin/(60*60)])

            subplot(2,2,3)
            scatter(so.t_trig_hr, so.sun_az,1.5)
            grid on;
            title('Sun Azimuth')
            xlabel('t (h)')
            ylabel('Azimuth Angle (deg)')
            xlim([0 tfin/(60*60)])
            ylim([0 360])

            subplot(2,2,4)
            plot(so.t_trig_hr, 90-so.sun_zth)
            grid on;
            title('Sun Elevation')
            xlabel('t (h)')
            ylabel('Elevation Angle (deg)')
            xlim([0 tfin/(60*60)])

            % plot power and energy data
            figure('Position',[100,100,1080,420]);
            subplot(1,2,1)
            grid on; hold on;
            plot(so.t_hr, so.P_el_in)
            plot(so.t_hr, so.P_el_out)
            plot(so.t_hr, so.P_batt)
            title('Power Flow')
            xlabel('t (h)')
            ylabel('Power (kW)')
            xlim([0 tfin/(60*60)])
            legend('Electric Power In', 'Electric Power Out', 'Battery Power In')

            subplot(1,2,2)
            plot(so.t_hr, so.E_batt)
            grid on; hold on;
            plot(so.t_hr, so.E_gravit)
            plot(so.t_hr, (so.E_batt+so.E_gravit))
            title('Energy Stored')
            xlabel('t (h)')
            ylabel('Energy (kWh)')
            xlim([0 tfin/(60*60)])
            legend('Battery Energy', 'Gravit. Energy (Rel)', 'Total Energy')


            % plot (and animate) the trajectory with color correlated to
            % equivalent airspeed
            th = 0:2*pi/500:2*pi;
            circ_x = 3000.*cos(th);
            circ_y = 3000.*sin(th);
            circ_z = obj.vehicle.alt_lim_l * ones(size(circ_y));
            figure('Color',[237/255, 238/255, 242/255], 'Position',[100, 100, 1080, 800]);
            plot3(circ_x, circ_y, circ_z, '--k', 'LineWidth',2);
            hold on; grid on; box;
            plot1 = scatter3(so.posn_east(1), so.posn_north(1), so.alt(1), 2, 0);
            xlim([-5500, 5500])
            ylim([-5500, 5500])
            zlim([18280, max(so.alt)+20])
            view(45, 30);
            title('Trajectory')
            xlabel('Y (m) [+ east]');
            ylabel('X (m) [+ north]');
            zlabel('h (m)');
            set(gca,'Color','none');
            set(gca,'CLim',[0, 1]);
            for k = 2:ceil(length(so.posn_east)/500):length(so.posn_east)
                 plot1.XData = so.posn_east(1:k);
                 plot1.YData = so.posn_north(1:k);
                 plot1.ZData = so.alt(1:k);
                 plot1.CData = (so.veas(1:k)-7.1)/(10.0-7.1);
                 %pause(0.01)
            end

            % Periodic Climb Analysis Plots
            if(isfield(obj.par_params,'alt_cmd_mode'))
                if(obj.par_params.alt_cmd_mode == 3)
                    figure('Position',[100,100,960,640]);
                    subplot(2,3,1)
                    plot(so.t_hr, so.P_el_out)
                    grid on;
                    xlabel('t (h)')
                    ylabel('Power (kW)')
                    title('Power Out')

                    subplot(2,3,2)
                    plot(so.t_hr, so.E_el_out)
                    grid on;
                    xlabel('t (h)')
                    ylabel('Energy (kWh)')
                    title('Energy Out')

                    subplot(2,3,3)
                    plot(so.t_hr, so.alt)
                    grid on;
                    xlabel('t (h)')
                    ylabel('Altitude (m)')
                    title('Altitude')

                    subplot(2,3,4)
                    plot(so.t_hr, so.P_el_in)
                    grid on;
                    xlabel('t (h)')
                    ylabel('Power (kW)')
                    title('Power In')

                    subplot(2,3,5)
                    plot(so.t_hr, so.E_el_in)
                    grid on;
                    xlabel('t (h)')
                    ylabel('Energy (kWh)')
                    title('Energy In')

                    subplot(2,3,6)
                    plot(so.t_trig_hr, 90-so.sun_zth)
                    grid on;
                    xlabel('t (h)')
                    ylabel('Sun Elevation (deg)')
                    title('Sun Elevation Angle')
                end
            end

            % Optimal Solution Plots
            if(opt_flag)

                if ~isempty(obj.opt_cmd)
                    ocmd = obj.opt_cmd;
                else
                    Error('Error: Commands for optimal solution are not saved');
                end

                % plot commands and actual values from simulation
                figure('Position',[100,100,1080,400]);
                subplot(1,3,1)
                plot(ocmd.alt_rt_cmd.Time/3600, ocmd.alt_rt_cmd.Data(:))
                grid on; hold on; plot(so.t_hr, so.vv)
                xlabel('t (h)')
                ylabel('Altitude Rate (m/s)')
                title('Altitude Rate Command')

                subplot(1,3,2)
                plot(ocmd.eas_cmd.Time/3600, ocmd.eas_cmd.Data(:))
                grid on; hold on; plot(so.t_hr, so.veas)
                xlabel('t (h)')
                ylabel('V_{EAS} (m/s)')
                title('Equivalent Airspeed Command')

                subplot(1,3,3)
                plot(ocmd.phi_cmd.Time/3600, ocmd.phi_cmd.Data(:) * obj.r2d)
                grid on; hold on; plot(so.t_hr, so.phi)
                xlabel('t (h)')
                ylabel('Bank Angle (deg)')
                title('Bank Angle Command')


                figure('Position',[100,100,1080,640]);
                subplot(2,3,1)
                plot(so.t_hr, so.alt)
                grid on;
                xlabel('t (h)')
                ylabel('Altitude (m)')
                title('Altitude')

                subplot(2,3,2)
                plot3(so.posn_east, so.posn_north, so.alt)
                grid on;
                xlabel('x (m)')
                ylabel('y (m)')
                zlabel('h (m)')
                title('Trajectory')

                subplot(2,3,3)
                plot(so.t_hr, so.vgnd)
                grid on; hold on; plot(so.t_hr, so.vtas)
                xlabel('t (h)')
                ylabel('V (m/s)')
                title('Ground Speed and TAS')

                subplot(2,3,4)
                plot(so.t_hr, so.p)
                grid on;
                xlabel('t (h)')
                ylabel('Bank Rate (deg/s)')
                title('Bank Rate: p')

                subplot(2,3,5)
                plot(so.t_hr, so.theta)
                grid on;
                xlabel('t (h)')
                ylabel('Pitch (deg)')
                title('Pitch Angle: \theta')

                subplot(2,3,6)
                plot(so.t_hr, so.chi)
                grid on;
                xlabel('t (h)')
                ylabel('Course (deg)')
                title('Course Angle: \chi')

            end

        end

        function [oE, sE] = compareSimAndOpt(obj)
            % [oE, sE] = compareSimAndOpt(obj)
            %   Compare energy storage of optimal and baseline solutions,
            %   taking advantage of the fact that a baseline simulation is
            %   always run before an optimization to initialize the
            %   optimizer.

            % make sure both optimization and baseline output structs exist
            so = obj.sim_out_parsed;
            if(isempty(so))
                error('Error: It appears that simulation has not been run/parsed yet');
            end
            oo = obj.opt_out_parsed;
            if(isempty(oo))
                error('Error: It appears that optimization has not been run/parsed yet');
            end

            oE = oo.E_stored;
            sE = so.E_stored;

            disp(['Energy stored from optimal trajectory: ', num2str(oE), ' kWh']);
            disp(['Energy stored from baseline trajectory: ', num2str(sE), ' kWh']);
            disp(['Energy stored from optimal trajectory is ', sprintf('%.2f',(oE/sE-1)*100), '% more than baseline trajectory.'])

        end

    end

    methods (Access = 'private')

        function trimAC(obj)
            % trimAC(obj)
            %   Take the supplied initial conditions and vehicle/aero
            %   parameters, and set the remaining initial
            %   conditions/parameters for the kinematic simulation

            % for shorthand - struct of initial conditions
            ic = obj.init_conds;

            % dynamic pressure
            q_bar = (0.5 * obj.rho_SL * ic.veas_init^2);

            % initial coefficient of lift
            ic.CL0 = obj.vehicle.M * obj.g / q_bar / obj.vehicle.S;

            % initial angle of attack
            ic.alpha_0 = EnergySim.init_alpha(ic.CL0, ic.alt_init, obj.aero.cl_data, obj.aero.alt_bp, obj.aero.alpha_bp);

            % initial coefficient of drag
            ic.CD0 = interp2(obj.aero.alt_bp, obj.aero.alpha_bp, obj.aero.cd_data, ic.alt_init, ic.alpha_0);

            % intiial thrust - from each motor
            ic.th_init = (obj.vehicle.M * obj.g * ic.CD0 / ic.CL0) / obj.vehicle.num_motors;

            % air density at altitude
            rho_init = obj.getRho();

            % initial true airspeed, ground speed, flight path angle
            [ic.vtas_init, ic.vgnd_init, ic.ga_init] = ...
                                        EnergySim.init_veloc(obj.rho_SL, ic.veas_init, ic.chi_init, [obj.wind.Wn obj.wind.We obj.wind.Wd], rho_init);

            % initial heading
            ic.psi_0 = ic.chi_init - asin((obj.wind.We * cos(ic.chi_init) - obj.wind.Wn * sin(ic.chi_init)) /  (ic.vtas_init * cos(ic.ga_init)));

            % initial pitch angle
            ic.theta_0 = ic.alpha_0 + ic.ga_init;

            % initial trim thrust
            ic.thrust0 = ic.CD0 * q_bar * obj.vehicle.S + ic.alt0_rate_init * obj.vehicle.M * obj.g / ic.vgnd_init;

            % replace stored initial conditions with the updated struct
            obj.init_conds = ic;

        end

        function rho_alt = getRho(obj)
            % rho_alt = getRho(obj)
            %   Calculate air density at the initial altitude. Simulink
            %   model "get_rho" uses library block from
            %   "atmospheric_library".

            rho_sim = Simulink.SimulationInput('get_rho');
            rho_sim = setVariable(rho_sim, 'dT', obj.dT);
            rho_sim = setVariable(rho_sim, 'alt_init', obj.init_conds.alt_init);
            simOut  = sim(rho_sim);

            % return air density at altitude
            rho_alt = simOut.get('rho_out').signals.values(end);

        end

        function opt_init = initBankOpt(obj, n)
            % bank_init = initBankOpt(obj, n)
            %   Call runSim to run a baseline simulation, and extract bank
            %   angle data to initialize the optimizer with. The sim_out
            %   struct created from running the simulation can also be used
            %   to compare optimal solution to baseline
                            % prepare, run, and parse simulation

            %The runSim method does the initialization of the simulink model parameters, so run it first no matter where the initial conditions come from
            % prepare, run, and parse simulation
            obj.runSim();

            t_evals   = linspace(0, obj.time.tfin, n+1);

            if (obj.init_conds.use_init_traj == true )

                if ( isempty(obj.init_conds.init_traj) )
                    error(' the "use_init_traj" flag is on; an initial trajectory shall be provided ');
                end

                % get the initial traj from the provided data file
                ES_i = load(obj.init_conds.init_traj);

                phi_ts        = ES_i.ES.opt_cmd.phi_cmd;
                dAlt_ts       = ES_i.ES.opt_cmd.alt_rt_cmd;

                %Convert the Data field into (n by 1) double array
                phi_ts.Data  = phi_ts.Data(:) ;
                dAlt_ts.Data = dAlt_ts.Data(:) ;
            else %if the use_init_traj == false


                phi   = obj.sim_out_parsed.y.signals.values(:,7);
                dAlt  = obj.sim_out_parsed.y.signals.values(:,16);
                t     = obj.sim_out_parsed.t;

                phi_ts  = timeseries(phi, t);
                dAlt_ts = timeseries(dAlt, t);

            end

                % prep bank angle commands to initalize optimizer
                opt_init.bank_init = resample(phi_ts, t_evals);
                opt_init.dAlt_init = resample(dAlt_ts, t_evals);

        end

        function si_ret = setSimOrOpt(obj, sim_in, opt_flag)
            % si_ret = setSimOrOpt(obj, sim_in, opt_flag)
            %   Update the Simulink.SimulationInput object ("sim_in") with
            %   parameters supplied to the instance of the EnergySim class.
            %   THey are all copied as variables to the Simulink model,
            %   rather than relying on whatever is in the base workspace
            %   for Simulink input. "opt_flag" is a flag
            %   used to differentiate between simulation and optimization.

            % for shorthand - Simulink.SimulationInput object to be updated
            % and then returned
            si = sim_in;

            % general params
            si = si.setVariable('dT', obj.dT);
            si = si.setVariable('dT_fixed', obj.dT_fixed);
            si = si.setVariable('dT_trig', obj.dT_trig);
            si = si.setVariable('d2r', obj.d2r);
            si = si.setVariable('r2d', obj.r2d);
            si = si.setVariable('earth_rho', obj.earth_rho);
            si = si.setVariable('rho_SL', obj.rho_SL);
            si = si.setVariable('g', obj.g);
            si = si.setVariable('WP_NAV', obj.wp_nav);
            si = si.setVariable('THRUST_REVERSAL', obj.thrust_rev);
            si = si.setVariable('Gsolar', obj.Gsolar);
            si = si.setVariable('alpha_h', obj.alpha_h);
            si = si.setVariable('absorb_h', obj.absorb_h);

            % test conditions
            si = si.setVariable('TEST_WINDS', obj.testOptions.autoWinds);
            si = si.setVariable('wmag_level', obj.testOptions.wmag_level);
            si = si.setVariable('wdir_level', obj.testOptions.wdir_level);
            si = si.setVariable('wind_day', obj.testOptions.dayIdx);
            si = si.setVariable('testStation', obj.testStation.tst_station);

            % winds
            w = obj.wind;
            si = si.setVariable('Wdir', w.Wdir);
            si = si.setVariable('Wmag', w.Wmag);
            si = si.setVariable('Wn', w.Wn);
            si = si.setVariable('We', w.We);
            si = si.setVariable('Wd', w.Wd);

            % initial conditions
            ic = obj.init_conds;
            if opt_flag
                si = si.setVariable('chi_init', ic.chi_init);
                si = si.setVariable('pN_init', ic.pN_init);
                si = si.setVariable('pE_init', ic.pE_init);
            end
            si = si.setVariable('alt_init', ic.alt_init);
            si = si.setVariable('phi_0', ic.phi_0);
            si = si.setVariable('lat_init_deg', ic.lat_init_deg);
            si = si.setVariable('lon_init_deg', ic.lon_init_deg);
            lat_init = ic.lat_init_deg * obj.d2r;
            lon_init = ic.lon_init_deg * obj.d2r;
            si = si.setVariable('lat_init', lat_init);
            si = si.setVariable('lon_init', lon_init);
            si = si.setVariable('chi0_rate_init', ic.chi0_rate_init);
            si = si.setVariable('alt0_rate_init', ic.alt0_rate_init);
            rho_init = obj.getRho();
            si = si.setVariable('rho_alt', rho_init);
            si = si.setVariable('E_init', ic.E_init);
            si = si.setVariable('veas_init', ic.veas_init);
            % initial conditions from trimming routine
            si = si.setVariable('CL0', ic.CL0);
            si = si.setVariable('CD0', ic.CD0);
            si = si.setVariable('alpha_0', ic.alpha_0);
            si = si.setVariable('psi_0', ic.psi_0);
            si = si.setVariable('theta_0', ic.theta_0);
            si = si.setVariable('thrust0', ic.thrust0);
            si = si.setVariable('th_init', ic.th_init);
            si = si.setVariable('vtas_init', ic.vtas_init);
            si = si.setVariable('vgnd_init', ic.vgnd_init);
            si = si.setVariable('ga_init', ic.ga_init);

            % vehicle
            v = obj.vehicle;
            si = si.setVariable('alt_lim_h', v.alt_lim_h);
            si = si.setVariable('alt_lim_l', v.alt_lim_l);
            w_xtrack = v.w_xtrack;
            if (~obj.wp_nav)
                w_xtrack = 2.0 * w_xtrack;
            end
            si = si.setVariable('z_xtrack', v.z_xtrack);
            si = si.setVariable('w_xtrack', w_xtrack);
            w_alt = v.w_alt;
            if (~obj.wp_nav)
                w_alt = 2.0 * w_alt;
            end
            si = si.setVariable('z_alt', v.z_alt);
            si = si.setVariable('w_alt', w_alt);
            if (obj.thrust_rev)
                thr_lo_lim = -9999.0;
            else
                thr_lo_lim = 0.0;
            end
            si = si.setVariable('thr_lo_lim', thr_lo_lim);
            si = si.setVariable('tau_as', v.tau_as);
            si = si.setVariable('veas_rt_lim', v.veas_rt_lim);
            si = si.setVariable('phi_lim', v.phi_lim);
            si = si.setVariable('phi_rt_lim', v.phi_rt_lim);
            si = si.setVariable('phi_lim_alt_min', v.phi_lim_alt_min);
            si = si.setVariable('phi_lim_alt_max', v.phi_lim_alt_max);
            si = si.setVariable('phi_lim_alt', [v.phi_lim_alt_min v.phi_lim_alt_max]);
            si = si.setVariable('gamma_lim', v.gamma_lim);
            si = si.setVariable('S', v.S);
            si = si.setVariable('b', v.b);
            si = si.setVariable('M', v.M);
            si = si.setVariable('AR', v.AR);
            si = si.setVariable('panel_vecs', v.panel_vecs);
            si = si.setVariable('panel_eff', v.panel_eff);
            si = si.setVariable('mppt_eff', v.mppt_eff);
            si = si.setVariable('P_acc', v.P_acc);
            Adisk = pi * v.R_prop^2;
            si = si.setVariable('R_prop', v.R_prop);
            si = si.setVariable('Adisk', Adisk);
            si = si.setVariable('num_motors', v.num_motors);
            si = si.setVariable('prop_eff_scale', v.prop_eff_scale);
            si = si.setVariable('motor_eff', v.motor_eff);
            si = si.setVariable('charge_eff', v.charge_eff);
            si = si.setVariable('discharge_eff', v.discharge_eff);
            si = si.setVariable('E_batt_max', v.E_batt_max);

            % time
            t = obj.time;
            si = si.setVariable('time', t.time_init);
            si = si.setVariable('time_init', t.t_init);
            si = si.setVariable('UTC', t.UTC);
            si = si.setVariable('tfin', t.tfin);

            % parametric sim params
            if ~opt_flag
                p = obj.par_params;
                [sun_az_0, sun_el_0, ~] = solar_vector(t.time_init, ic.lat_init_deg, ic.lon_init_deg, ic.alt_init);
                fp = solarFP(p.orbit_rho, p.min_axis, p.orbit_dir, ic.x_ctr, ic.y_ctr, sun_az_0);
                if p.orbit_start && (p.chi_cmd_mode == 5)
                    az_init = sun_az_0;
                    si = si.setVariable('chi_init', az_init);
                    pN_init = (fp(1,4) + fp(fp(1,2),4))/2;
                    pE_init = (fp(1,5) + fp(fp(1,2),5))/2;
                    si = si.setVariable('pN_init', pN_init);
                    si = si.setVariable('pE_init', pE_init);
                elseif p.orbit_start && (p.chi_cmd_mode == 4)
                    if (p.orbit_dir > 0)
                        az_init = sun_az_0 - pi/2;
                    else
                        az_init = sun_az_0 + pi/2;
                    end
                    si = si.setVariable('chi_init', az_init);
                    pN_init = p.orbit_rho * cos(sun_az_0);
                    pE_init = p.orbit_rho * sin(sun_az_0);
                    si = si.setVariable('pN_init', pN_init);
                    si = si.setVariable('pE_init', pE_init);
                else
                    si = si.setVariable('chi_init', ic.chi_init);
                    pN_init = ic.pN_init;
                    pE_init = ic.pE_init;
                    si = si.setVariable('pN_init', pN_init);
                    si = si.setVariable('pE_init', pE_init);
                end
                [ICs, fp_courses, fp_coords] = EnergySim.msnInit(fp, 1, pN_init, pE_init);
                [num_wp,~] = size(fp);
                ND2 = load(fullfile(pwd,'nd2_data.mat'));
                si = si.setVariable('min_axis', p.min_axis);
                si = si.setVariable('chi_cmd_mode', p.chi_cmd_mode);
                si = si.setVariable('alt_rt_lim_periodic', p.alt_rt_lim_periodic);
                si = si.setVariable('alt_cmd_mode', p.alt_cmd_mode);
                si = si.setVariable('sel_vec', zeros(1, num_wp));
                si = si.setVariable('sun_az_0', sun_az_0);
                si = si.setVariable('sun_el_0', sun_el_0);
                si = si.setVariable('to_init', ICs(1));
                si = si.setVariable('next_init', ICs(2));
                si = si.setVariable('d_init', ICs(3));
                si = si.setVariable('dchi0', ICs(4));
                si = si.setVariable('tan_phi_lim', tan(10*pi/180));
                si = si.setVariable('orb_dir_init', fp(ICs(1),8));
                si = si.setVariable('orb_len_init', fp(ICs(1),9));
                si = si.setVariable('entry_init', fp(ICs(1),10));
                si = si.setVariable('exit_init', fp(ICs(1),11));
                si = si.setVariable('fp0', fp);
                si = si.setVariable('ND2', ND2.ND2);
                si = si.setVariable('fp_courses0', fp_courses);
                si = si.setVariable('fp_coords0', fp_coords);
                si = si.setVariable('t_fly_by', 2.0);
                si = si.setVariable('chi_inf', 1.5707963267949);
                si = si.setVariable('k', 1e-3);
                si = si.setVariable('epsln', 0.785398163397448);
                si = si.setVariable('gamma', 1.1);
                si = si.setVariable('K', 14254.9766656637);
                si = si.setVariable('alpha', 7127.48833283185);
                si = si.setVariable('epsln_circ', 40*pi/180);
                si = si.setVariable('k_c', 5);
                si = si.setVariable('K_circ', 453.6);
                si = si.setVariable('alpha_circ', 453.6 / 1.2);
                si = si.setVariable('x_ctr', ic.x_ctr);
                si = si.setVariable('y_ctr', ic.y_ctr);
                si = si.setVariable('orbit_dir', p.orbit_dir);
                si = si.setVariable('orbit_rho', p.orbit_rho);
                si = si.setVariable('turn_time_circle', p.turn_time_circle);
                si = si.setVariable('turn_time_rt', p.turn_time_rt);
                si = si.setVariable('hold_time', p.hold_time);
                si = si.setVariable('rt_azimuth_offset', p.rt_azimuth_offset);
                si = si.setVariable('fig_eight_time', p.fig_eight_time);
                si = si.setVariable('f8_azimuth_offset', p.f8_azimuth_offset);
                si = si.setVariable('veas_night_mode', p.veas_night_mode);
                si = si.setVariable('enableFuzzyLogic', p.enableFuzzyLogic);
            else
                si = si.setVariable('chi_cmd_mode', 0);
            end

            % aero
            a = obj.aero;
            si = si.setVariable('alt_bp', a.alt_bp);
            si = si.setVariable('alpha_bp', a.alpha_bp);
            si = si.setVariable('cl_data', a.cl_data);
            si = si.setVariable('cd_data', a.cd_data);
            si = si.setVariable('max_cl', a.max_cl);
            si = si.setVariable('min_cl', a.min_cl);
            si = si.setVariable('max_alt_bp', a.max_alt_bp);
            si = si.setVariable('min_alt_bp', a.min_alt_bp);
            si = si.setVariable('max_alpha_bp', a.max_alpha_bp);
            si = si.setVariable('min_alpha_bp', a.min_alpha_bp);
            si = si.setVariable('cl_bp', a.cl_bp);
            si = si.setVariable('aoa_data', a.aoa_data);

            % filtering
            si = si.setVariable('sys_diff_A', obj.sys_diff.A);
            si = si.setVariable('sys_diff_B', obj.sys_diff.B);
            si = si.setVariable('sys_diff_C', obj.sys_diff.C);
            si = si.setVariable('sys_diff_D', obj.sys_diff.D);
            si = si.setVariable('sys_diff_x0', obj.sys_diff.x0);
            si = si.setVariable('sys_lpf_A', obj.sys_lpf.A);
            si = si.setVariable('sys_lpf_B', obj.sys_lpf.B);
            si = si.setVariable('sys_lpf_C', obj.sys_lpf.C);
            si = si.setVariable('sys_lpf_D', obj.sys_lpf.D);
            si = si.setVariable('sys_lpf_x0', obj.sys_lpf.x0);

            si_ret = EnergySim.setSigs(si);

        end

        function si_ret = setCmds(obj, sim_in, alt_rt, eas, phi)
            % si_ret = setCmds(obj, sim_in, alt_rt, eas, phi)
            %   Update the Simulink.SimulationInput object ("sim_in") with
            %   commands supplied as inputs to this function, and return
            %   the updated Simulink.SimulationInput object. Also store the
            %   updated object as obj.opt_cmd.

            si = sim_in;
        	si = si.setVariable('alt_rt_cmd', alt_rt);
        	si = si.setVariable('eas_cmd', eas);
        	si = si.setVariable('phi_cmd', phi);

            % copy commands to obj.opt_cmd
            obj.opt_cmd.alt_rt_cmd = alt_rt;
            obj.opt_cmd.eas_cmd = eas;
            obj.opt_cmd.phi_cmd = phi;

            % return Simulink.SimulationInput object
            si_ret = si;

        end

        function parsed = parseSim(obj, sim_out, opt_flag)
            % parsed = parseSim(obj, sim_out, opt_flag)
            %   Take a Simulink.SimulationOutput object, extract all the
            %   desired signals, and rename as desired.

            % for shorthand - Simulink.SimulationOutput object to be parsed
            so = sim_out;

            % make sure desired object isn't empty
            if(isempty(so))
                error('Error: It appears that simulation has not been run yet');
            end

            % get signal vectors from simulation
            t = so.get('tout');
            y = so.get('yout');
            pwr = so.get('pwr_mdl_out');
            wnd = so.get('wind_state_out');
            trainingData = so.get('train_states_out');

            % sanity check
            if(t(end) ~= obj.time.tfin)
                error(['Error: Time from simulation output (',num2str(t(end)),') does not match desired sim time (', num2str(obj.time.tfin),')']);
            end

            % create struct (sim out parsed) and copy sim_out signals
            so_p.t = t;     % time (s)
            so_p.y = y;     % av state
            so_p.pwr = pwr; % power state
            so_p.wnd = wnd;
%             so_p.trainingData = trainingData; % training data

            so_p.t_hr = t/(60*60);
            %Training Data
%             so_p.dAz   = trainingData.signals.values(:,1);
%             so_p.dChi  = trainingData.signals.values(:,2);
%             so_p.dAlt  = trainingData.signals.values(:,3);
            %E_batt is stored from the power data
            %wind is also stored from the states data

            % Winds
            so_p.wnd_n = wnd.signals.values(:,1);
            so_p.wnd_e = wnd.signals.values(:,2);
            so_p.wnd_d = wnd.signals.values(:,3);
            % Axial speeds
            so_p.u = y.signals.values(:,1);
            so_p.v = y.signals.values(:,2);
            so_p.w = y.signals.values(:,3);
            % Axial angular rates
            so_p.p = y.signals.values(:,4) * obj.r2d;
            so_p.q = y.signals.values(:,5) * obj.r2d;
            so_p.r = y.signals.values(:,6) * obj.r2d;
            % Euler angles
            so_p.phi = y.signals.values(:,7) * obj.r2d;
            so_p.theta = y.signals.values(:,8) * obj.r2d;
            so_p.psi = y.signals.values(:,9) * obj.r2d;
            % Position
            so_p.posn_north = y.signals.values(:,10);
            so_p.posn_east = y.signals.values(:,11);
            so_p.alt = y.signals.values(:,12);
            so_p.av_lat_deg = y.signals.values(:,27);
            so_p.av_lon_deg = y.signals.values(:,28);
            % Speeds
            so_p.veas = y.signals.values(:,13);  % equivalent veloc
            so_p.vtas = y.signals.values(:,14);  % true veloc
            so_p.vgnd = y.signals.values(:,15);  % ground veloc
            so_p.vv = y.signals.values(:,16);    % vertical veloc
            so_p.qbar = y.signals.values(:,25);  % dynamic pressure
            so_p.mach = y.signals.values(:,26);  % Mach
            % Wind angles
            so_p.alpha = y.signals.values(:,17) * obj.r2d;
            so_p.beta = y.signals.values(:,18) * obj.r2d;
            so_p.gamma = y.signals.values(:,19) * obj.r2d; % flight path
            % Inertial Angles
            so_p.fpa = y.signals.values(:,20) * obj.r2d;   % inertial flight path
            so_p.chi = y.signals.values(:,21) * obj.r2d;   % course
            % Forces
            so_p.CL = y.signals.values(:,22);
            so_p.CD = y.signals.values(:,23);
            so_p.thrust = y.signals.values(:,24);

            if(~isempty(pwr))

                % for back compatibility with older version
                if pwr.signals.dimensions == 15

                    % Powers - all kW
                    so_p.P_el_in     = pwr.signals.values(:,4);   % solar power (after panels)
                    so_p.P_el_out    = pwr.signals.values(:,5);   % power drawn by motors and appliacnces
                    so_p.P_el_net    = so_p.P_el_in-so_p.P_el_out;
                    so_p.P_batt      = pwr.signals.values(:,12);  % net power into storage (after charging/discharging efficiency penalty)
                    % Energies - all kWh
                    so_p.E_el_in     = pwr.signals.values(:,6);   % gross solar energy
                    so_p.E_el_out    = pwr.signals.values(:,7);   % integral of electric power used
                    so_p.E_batt      = pwr.signals.values(:,13);  % energy stored in battery
                    so_p.E_gravit    = pwr.signals.values(:,14);  % energy stored by climbing
                    % Solar parameters
                    so_p.sun_az     = pwr.signals.values(:,1);    % solar azimuth angle (deg)
                    so_p.sun_zth    = pwr.signals.values(:,2);    % solar zenith angle (deg)
                    so_p.jd         = pwr.signals.values(:,3);    % Julian day
                    so_p.G_adjusted = pwr.signals.values(:,9);    % JD- and atmos-adjusted solar "constant" (W/m^2)

                    if (obj.wp_nav)
                        so_p.alt_cmd   = so.get('alt_cmd');             % altitude command
                        so_p.chi_cmd   = so.get('chi_cmd') * obj.r2d;   % course command (deg)
                    end
                    so_p.prop_eff  = pwr.signals.values(:,10);    % propeller efficiency
                    so_p.proj_area = pwr.signals.values(:,8);     % projected area of panels normal to sun

                    so_p.t_trig = t;
                    so_p.t_trig_hr = so_p.t_trig/(60*60);

                else

                    % Powers - all kW
                    so_p.P_el_in     = pwr.signals.values(:,4);   % solar power (after panels)
                    so_p.P_el_out    = pwr.signals.values(:,5);   % power drawn by motors and appliacnces
                    so_p.P_el_net    = so_p.P_el_in-so_p.P_el_out;
                    so_p.P_batt      = pwr.signals.values(:,6);   % net power into storage (after charging/discharging efficiency penalty)
                    % Energies - all kWh
                    so_p.E_el_in     = pwr.signals.values(:,7);   % gross solar energy
                    so_p.E_el_out    = pwr.signals.values(:,8);   % integral of electric power used
                    so_p.E_batt      = pwr.signals.values(:,9);   % energy stored in battery
                    so_p.E_gravit    = pwr.signals.values(:,10);  % energy stored by climbing - note that this is the delta energy w.r.t the station keeping floor
                    % Solar parameters
                    so_p.sun_az     = pwr.signals.values(:,1);    % solar azimuth angle (deg)
                    so_p.sun_zth    = 90 - pwr.signals.values(:,2);    % solar zenith angle (deg)
                    so_p.jd         = pwr.signals.values(:,3);    % Julian day
                    so_p.G_adjusted = pwr.signals.values(:,13);   % JD- and atmos-adjusted solar "constant" (W/m^2)

                    if (obj.wp_nav)
                        so_p.alt_cmd   = so.get('alt_cmd');             % altitude command
                        so_p.chi_cmd   = so.get('chi_cmd') * obj.r2d;   % course command (deg)
                    end
                    so_p.prop_eff  = pwr.signals.values(:,17);    % propeller efficiency
                    so_p.proj_area = pwr.signals.values(:,12);    % projected area of panels normal to sun

                    so_p.t_trig = t;
                    so_p.t_trig_hr = so_p.t_trig/(60*60);

                end
            end

            % commands
            if opt_flag
                so_p.alt_rt_cmd = obj.opt_cmd.alt_rt_cmd;
                so_p.eas_cmd = obj.opt_cmd.eas_cmd;
                so_p.phi_cmd = obj.opt_cmd.phi_cmd;
            end

            % difference in (battery+gravitational) energy over time period
            so_p.E_stored = so_p.E_batt(end)+so_p.E_gravit(end)-so_p.E_batt(1)-so_p.E_gravit(1);

            % index of 24h before end of sim (if longer than 24h)
            so_p.i_end_minus_24h = find(so_p.t_hr>so_p.t_hr-24,1);

            so_p.E_batt_last_day = so_p.E_batt(so_p.i_end_minus_24h:end);

            % minimum state of charge (percentage) over last 24h of sim
            so_p.min_soc = 100*min(so_p.E_batt_last_day)/obj.vehicle.E_batt_max;

            % return filled in struct
            parsed = so_p;
            disp('Simulation output parsed.');

        end

        function ss_sys = tfToSS(obj, f, diff_flag)
            % ss_sys = tfToSS(obj, f, diff_flag)
            %   Take a continuous-time transfer function and convert to
            %   discrete-time state space system. "f" is desired frequency
            %   in Hz, and "diff_flag" is a flag for differentiating (true)
            %   or low-pass filtering (false).

            % either differentiation or low-pass filtering
            if diff_flag
                sys_c = ss(tf([1 0],[1/(2*pi*f) 1]));
            else
                sys_c = ss(tf([0 1],[1/(2*pi*f) 1]));
            end

            % compute discrete SS system from continuous
            sys_d    = c2d(sys_c, obj.dT);
            ss_sys.A  = sys_d.A;
            ss_sys.B  = sys_d.B;
            ss_sys.C  = sys_d.C;
            ss_sys.D  = sys_d.D;
            ss_sys.x0 = -sys_c.B / sys_c.A;

        end

        function ret = runSimWithOpt(obj, opt_in)
            % ret = runSimWithOpt(obj, opt_in)
            %   Run the optimization Simulink model with the commands
            %   output from the optimizer as the optimal solution. This
            %   allows for the extraction of signals from the simulation.
            %   "opt_in" is the Simulink.SimulationInput instance to be
            %   simulated.

            % for shorthand - number of time intervals
            n = obj.opt_params.n;

            % split vector returned by optimizer into optimal commands
            opt_alt_rt = obj.opt_data_out.X(1:n);

            if (obj.opt_params.optVeas == true)
                opt_eas    = obj.opt_data_out.X(n+1:2*n);
                opt_phi    = obj.opt_data_out.X(2*n+1:end);
            else
                opt_eas   =  obj.init_conds.veas_init * ones(1,n) ;
                opt_phi   =  obj.opt_data_out.X(n+1:end) ;
            end

            % add initial conditions and timings to command vectors
            t_evals      = linspace(0, obj.time.tfin, n+1);
            alt_rt_evals = [obj.init_conds.alt0_rate_init, opt_alt_rt];
            veas_evals   = [obj.init_conds.veas_init,      opt_eas   ];
            phi_evals    = [obj.init_conds.phi_0,          opt_phi   ];

            % convert vectors into timeseries (linearly interpolating too)
            alt_rt_cmd = timeseries(alt_rt_evals, t_evals);
            eas_cmd    = timeseries(veas_evals,   t_evals);
            phi_cmd    = timeseries(phi_evals,    t_evals);

            % add the commands as variables to the SimIn object
            oi_fin = obj.setCmds(opt_in, alt_rt_cmd, eas_cmd, phi_cmd);

            disp('Simulation of optimum has begun.')
            start_timer = tic;

            % run the simulation
            ret = sim(oi_fin);

            stop_timer = toc(start_timer);
            disp(['Simulation completed in ',num2str(stop_timer),' seconds.'])

        end

        function opt_out = optRoutine(obj, X0, opt_in)
            % opt_out = optRoutine(obj, X0, opt_in)
            %   This function is a wrapper around the optimization routine,
            %   which takes a Simulink.SimulationInput object and vector of
            %   initial optimization variables, and fills in bounds and
            %   options from EnergySim object. Optimization solver fmincon
            %   could be replaced here by other solvers.

            % make an anonymous function to convert the objective function
            % to function of a single input (vector X - requiredd for fmincon)
            fun = @(X) obj.objFun(X, opt_in);

            % run fmincon - interior point method
            if (obj.opt_params.opt_opt == 0)

                % set options for fmincon
                fmc_opts = optimoptions('fmincon','Display','iter','UseParallel',obj.opt_params.pll,'MaxFunctionEvaluations',obj.opt_params.fevals);
                [X,f,eflag,outpt] = fmincon(fun,X0,[],[],[],[],obj.opt_params.lb,obj.opt_params.ub,[],fmc_opts);

            elseif (obj.opt_params.opt_opt == 1)

                % set options
                fsb_opts = optimset('Display','iter','UseParallel',obj.opt_params.pll,'MaxFunEvals',obj.opt_params.fevals);
                [X,f,eflag,outpt] = obj.simplexSearchNMbnd(fun,X0,obj.opt_params.lb,obj.opt_params.ub,fsb_opts,[]);

            elseif (obj.opt_params.opt_opt == 2)

                % In this method, we transform the time-domain variables
                % into Fourier Series, and apply optimization technique
                % over the coefficients of Fourier Series.

                % This method is NOT compatible with obj.opt_params.optVeas = false
                if obj.opt_params.optVeas == false
                  error('Fourier Transformation need to include Veas as optimization variable, please set optVeas = true.');
                end

                N_fft_order = obj.opt_params.fft_order;

                x0_fft = obj.fft_initial_condition(X0, N_fft_order);

                fft_fun = @(X) obj.fft_objfun(X, N_fft_order, opt_in);

                fmc_opts = optimoptions('fmincon','Display','iter','UseParallel',obj.opt_params.pll,'MaxFunctionEvaluations',obj.opt_params.fevals);

                nonlcon = @(X) obj.non_linear_con(X, N_fft_order);

                % Optimization algorithm can be changed here to optimize over coefficients of Fourier Series
                [X_coeff,f,eflag,outpt] = fmincon(fft_fun,x0_fft,[],[],[],[],[],[],nonlcon,fmc_opts);

                % transform fourier coefficient to commands
                X = obj.coeff_to_commands(X_coeff, N_fft_order);

            elseif (obj.opt_params.opt_opt == 3)

                % genetic algorithm

                % set options for genetic algorithm (ga)
                num_vars = length(X0);
                opts = optimoptions(@ga,...
                                    'display','iter',...
                                    'UseParallel',obj.opt_params.pll,...
                                    'MaxGenerations',500,...
                                    'MaxStallGenerations',20,...
                                    'PopulationSize',num_vars*2,...
                                    'InitialPopulation', X0,...
                                    'SelectionFcn',{@selectionroulette},...
                                    'MutationFcn',{@mutationadaptfeasible});
                disp('Genetic algorithm starts!');
                [X,f,eflag,outpt] = ga(fun,num_vars,[],[],[],[],obj.opt_params.lb,obj.opt_params.ub,[],opts);

            elseif (obj.opt_params.opt_opt == 4)

                % custom gradient descent with projection

                % add the folder path of the steepest descent solver
                addpath('../steepest_descent');

                opts = {'max_iter', ceil(obj.opt_params.fevals/(length(X0))), 'grad_tol', 1.0e-3, 'grad_delta', 1.0e-6, 'step_size_tol', 1.0e-10, 'use_parallel', obj.opt_params.pll};
                [X, f, eflag] = steepest_descent(fun, X0, obj.opt_params.lb, obj.opt_params.ub, opts);
                outpt = [];

            else
                disp('Not defined optimization routine type')
            end

            % optimal set of commands (3n by 1)
            opt_out.X = X;

            % objective function value
            opt_out.f = f;

            % exit flag from fmincon
            opt_out.exitflag  = eflag;

            % struct of optimization output (iterations, fx calls, etc.)
            opt_out.output = outpt;

        end

        % ======================================
        % ======== subfunctions for FFT ========
        % ======================================
        function [a,b,yfit] = Fseries(obj, x,y,n,scale,sincos)
            % Reference: https://www.mathworks.com/matlabcentral/fileexchange/31013-simple-real-fourier-series-approximation

            % FSERIES Computes real Fourier series approximation to a data set
            %
            % [A,B] = FSERIES(X,Y,N) fits an Nth order Fourier expansion of the form
            %    y = A_0/2 + Sum_k[ A_k cos(kx) + B_k sin(kx) ]
            % to the data in the vectors X & Y, using a least-squares fit.
            %
            % [A,B,YFIT] = FSERIES(X,Y,N) also returns a vector YFIT of the y values
            % obtained by evaluating the fitted model at the values in X.
            %
            % [A,B,YFIT] = FSERIES(X,Y,N,RESCALING) scales the X data to lie in the
            % interval [-pi,pi] if RESCALING is TRUE (default).  If RESCALING is
            % FALSE, no rescaling of X is performed.
            %
            % [A,B,YFIT] = FSERIES(X,Y,N,RESCALING,TYPE) uses a sine expansion if TYPE
            % is 'sin' or a cosine expansion if TYPE is 'cos'.  Both A & B are still
            % returned, however A will be all zero if TYPE = 'sin' and B will be all
            % zero if TYPE = 'cos'.
            %
            % See also: Fseriesval

            if nargin<3
                error('MATLAB:Fseries:MissingInputs','Required inputs are x, y, and n')
            end
            checkinputs();

            % scale x to [-pi,pi]
            if scale
                x1 = min(x);
                x2 = max(x);
                x = pi*(2*(x-x1)/(x2-x1) - 1);
            end

            % make design matrix
            nx = x*(1:n);
            if isequal(sincos,'b')
                F = [0.5*ones(size(x)),cos(nx),sin(nx)];
            elseif isequal(sincos,'c')
                F = [0.5*ones(size(x)),cos(nx)];
            else
                F = sin(nx);
            end

            % do fit
            c = F\y;

            % extract coefficients
            if isequal(sincos,'b')
                a = c(1:n+1);
                b = c(n+2:end);
            elseif isequal(sincos,'c')
                a = c;
                b = zeros(n,1);
            else
                a = zeros(n+1,1);
                b = c;
            end

            % evaluate fit
            yfit = F*c;

            % transpose yfit back to a row, if y was a row
            if xrow
                yfit = yfit';
            end


            function checkinputs
                % x & y values
                if isnumeric(x) && isvector(x) && isnumeric(y) && isvector(y)
                    if ~isequal(size(x),size(y))
                        throwAsCaller(MException('MATLAB:Fseries:UnequalData','x and y must be same size'))
                    end
                    % transpose x & y to columns if they are rows
                    if size(x,2)>1
                        x = x';
                        y = y';
                        xrow = true;
                    else
                        xrow = false;
                    end
                    % remove anything not finite and real
                    idx = ~(isfinite(x) & isfinite(y) & isreal(x) & isreal(y));
                    x(idx) = [];
                    y(idx) = [];
                    if isempty(x)
                        throwAsCaller(MException('MATLAB:Fseries:NoData','x and y contain no real, finite data'))
                    end
                else
                    throwAsCaller(MException('MATLAB:Fseries:WrongDataType','x and y values must be numeric vectors'))
                end
                % number of terms
                if isscalar(n) && isnumeric(n)
                    if n<0
                        throwAsCaller(MException('MATLAB:Fseries:NegativeTerms','Number of terms (n) cannot be negative!'));
                    end
                else
                    throwAsCaller(MException('MATLAB:Fseries:WrongDataType','n must be a numeric scalar'))
                end
                % optional scaling argument
                if exist('scale','var')
                    if ~islogical(scale)
                        throwAsCaller(MException('MATLAB:Fseries:WrongDataType','Scaling parameter must be logical (true/false)'))
                    end
                else
                    scale = true;
                end
                % optional type argument
                if exist('sincos','var')
                    if ischar(sincos)
                        sincos = lower(sincos);
                        if ismember(sincos,{'s','sin','sine'})
                            sincos = 's';
                        elseif ismember(sincos,{'c','cos','cosine'})
                            sincos = 'c';
                        else
                            throwAsCaller(MException('MATLAB:Fseries:WrongFourierType',['Unknown type: ',sincos]))
                        end
                    else
                        throwAsCaller(MException('MATLAB:Fseries:WrongDataType','Type must be string (''sin'' or ''cos'')'))
                    end
                else
                    sincos = 'b';
                end
            end

        end

        function y = Fseriesval(obj, a,b,x,scale)
            % Reference: https://www.mathworks.com/matlabcentral/fileexchange/31013-simple-real-fourier-series-approximation

            % FSERIESVAL Evaluates real Fourier series approximation at given data values
            %
            % Y = FSERIESVAL(A,B,X) the Fourier expansion of the form
            %    y = A_0/2 + Sum_k[ A_k cos(kx) + B_k sin(kx) ]
            % at the data values in the vector X.
            %
            % Y = FSERIESVAL(A,B,X,RESCALING) scales the X data to lie in the interval
            % [-pi,pi] if RESCALING is TRUE (default).  If RESCALING is FALSE, no
            % rescaling of X is performed.
            %
            % See also: Fseries

            if nargin<3
                error('MATLAB:Fseriesval:MissingInputs','Required inputs are a, b, and x')
            end
            checkinputs();

            % scale x to [-pi,pi]
            if scale
                x1 = min(x);
                x2 = max(x);
                x = pi*(2*(x-x1)/(x2-x1) - 1);
            end

            % make design matrix
            nx = x*(1:n);
            F = [0.5*ones(size(x)),cos(nx),sin(nx)];

            % evaluate fit
            y = F*[a;b];

            % transpose y back to a row, if x was a row
            if xrow
                y = y';
            end


            function checkinputs
                % coefficients
                if isnumeric(a) && isvector(a) && isnumeric(b) && isvector(b)
                    % get number of terms in F series
                    n = length(b);
                    if length(a) ~= (n+1)
                        throwAsCaller(MException('MATLAB:Fseriesval:InconsistentCoeffs','Inconsistent coefficient vectors'))
                    end
                else
                    throwAsCaller(MException('MATLAB:Fseriesval:WrongDataType','Coefficients must be numeric vectors'))
                end
                % x values
                if isnumeric(x) && isvector(x)
                    % transpose x to a column if it is a row
                    if size(x,2)>1
                        x = x';
                        xrow = true;
                    else
                        xrow = false;
                    end
                else
                    throwAsCaller(MException('MATLAB:Fseriesval:WrongDataType','x values must be a numeric vector'))
                end
                % optional scaling argument
                if exist('scale','var')
                    if ~islogical(scale)
                        throwAsCaller(MException('MATLAB:Fseriesval:WrongDataType','Scaling parameter must be logical (true/false)'))
                    end
                else
                    scale = true;
                end
            end

        end

        function x_fft = fft_initial_condition(obj, x0, N_fft_order)
            % Use least-squared fit to get initial Fourier Transormation
            % coefficients from baseline solution, which will be used as
            % the initial solution

            x_fft = [];

            % for shorthand - number of time intervals
            n = obj.opt_params.n;

            % least-squared fit to find coefficients of
            % alt_rt, Veas, Bank angle commands
            t = linspace(0, 1, n);
            [A, B] = obj.Fseries(t, x0(1:n),       N_fft_order(1));
            x_fft = [x_fft, A', B'];
            [A, B] = obj.Fseries(t, x0(n+1:2*n),   N_fft_order(2));
            x_fft = [x_fft, A', B'];
            [A, B] = obj.Fseries(t, x0(2*n+1:end), N_fft_order(3));
            x_fft = [x_fft, A', B'];

        end

        function f = fft_objfun(obj, X, N_order, opt_in)
            % Calculate the value of objective function,
            % given the Fourier Series coefficient.
            % X is the coefficients of the Fourier Series.

            % Get aircraft commands from Fourier Series Coefficients
            X_commands = obj.coeff_to_commands(X, N_order);

            % Evaluate objective function
            f = obj.objFun(X_commands, opt_in);

        end

        function [c, ceq] = non_linear_con(obj, X, N_order)
            % Produce non-linear constraints for optimization routine
            % when Fourier Transformation is enabled

            % Get aircraft commands from Fourier Series Coefficients
            X_commands = obj.coeff_to_commands(X, N_order);

            c = [X_commands - obj.opt_params.ub; -X_commands + obj.opt_params.lb];

            ceq = [];
        end

        function X_commands = coeff_to_commands(obj, X, N_order)
            % Given coefficients of the Fourier Series,
            % find the time-domain variables, i.e., aircraft commands.
            % X is the coefficients of the Fourier Series

            t = linspace(0 + 1.0/obj.opt_params.n, 1, obj.opt_params.n);

            alt_x = X(1 : 2*N_order(1)+1);
            alt_cmd = obj.Fseriesval(alt_x(1 : N_order(1)+1)', alt_x(N_order(1)+2 : end)', t);

            eas_x = X(2*N_order(1)+2 : 2*N_order(1)+2*N_order(2)+2);
            eas_cmd = obj.Fseriesval(eas_x(1 : N_order(2)+1)', eas_x(N_order(2)+2 : end)', t);

            phi_x = X(2*N_order(1)+2*N_order(2)+3 : end);
            phi_cmd = obj.Fseriesval(phi_x(1 : N_order(3)+1)', phi_x(N_order(3)+2 : end)', t);

            X_commands = [alt_cmd, eas_cmd, phi_cmd];
        end
        % ======================================
        % ==== end subfunctions for FFT ========
        % ======================================


        % ======================================
        % ========= begin subfunctions =========
        % ======================================
        function fval = intrafun(obj,x,params)
            % transform variables, then call original function

            % transform
            xtrans = obj.xtransform(x,params);

            % and call fun
%             fval = feval(params.fun,xtrans,params.args{:});
            fval = feval(params.fun,xtrans);

        end % sub function intrafun end

        % ======================================
        function xtrans = xtransform(obj,x,params)
        % converts unconstrained variables into their original domains

            xtrans = zeros(1,params.n);
            % k allows some variables to be fixed, thus dropped from the
            % optimization.
            k=1;
            for i = 1:params.n
              switch params.BoundClass(i)
                case 1
                  % lower bound only
                  xtrans(i) = params.LB(i) + x(k).^2;

                  k=k+1;
                case 2
                  % upper bound only
                  xtrans(i) = params.UB(i) - x(k).^2;

                  k=k+1;
                case 3
                  % lower and upper bounds
                  xtrans(i) = (sin(x(k))+1)/2;
                  xtrans(i) = xtrans(i)*(params.UB(i) - params.LB(i)) + params.LB(i);
                  % just in case of any floating point problems
                  xtrans(i) = max(params.LB(i),min(params.UB(i),xtrans(i)));

                  k=k+1;
                case 4
                  % fixed variable, bounds are equal, set it at either bound
                  xtrans(i) = params.LB(i);
                case 0
                  % unconstrained variable.
                  xtrans(i) = x(k);

                  k=k+1;
              end
            end

       end % sub function xtransform end


       function [x,fval,exitflag,output]=simplexSearchNMbnd(obj,fun,x0,LB,UB,options,varargin)
            % size checks
            xsize = size(x0);
            x0 = x0(:);
            n=length(x0);

            if (nargin<3) || isempty(LB)
              LB = repmat(-inf,n,1);
            else
              LB = LB(:);
            end
            if (nargin<4) || isempty(UB)
              UB = repmat(inf,n,1);
            else
              UB = UB(:);
            end

            if (n~=length(LB)) || (n~=length(UB))
              error 'x0 is incompatible in size with either LB or UB.'
            end

            % set default options if necessary
            if (nargin<5) || isempty(options)
              options = optimset('fminsearch');
            end

            % stuff into a struct to pass around
            params.args = varargin;
            params.LB = LB;
            params.UB = UB;
            params.fun = fun;
            params.n = n;
            params.OutputFcn = [];

            % 0 --> unconstrained variable
            % 1 --> lower bound only
            % 2 --> upper bound only
            % 3 --> dual finite bounds
            % 4 --> fixed variable
            params.BoundClass = zeros(n,1);
            for i=1:n
              k = isfinite(LB(i)) + 2*isfinite(UB(i));
              params.BoundClass(i) = k;
              if (k==3) && (LB(i)==UB(i))
                params.BoundClass(i) = 4;
              end
            end

        % transform starting values into their unconstrained
        % surrogates. Check for infeasible starting guesses.
        x0u = x0;
        k=1;
        for i = 1:n   % n = 6 number of param which needs to be optimisied
          switch params.BoundClass(i)
            case 1
              % lower bound only
              if x0(i)<=LB(i)
                % infeasible starting value. Use bound.
                x0u(k) = 0;
              else
                x0u(k) = sqrt(x0(i) - LB(i));
              end

              % increment k
              k=k+1;
            case 2
              % upper bound only
              if x0(i)>=UB(i)
                % infeasible starting value. use bound.
                x0u(k) = 0;
              else
                x0u(k) = sqrt(UB(i) - x0(i));
              end

              % increment k
              k=k+1;
            %% This is the bounded case
            case 3
              % lower and upper bounds
              if x0(i)<=LB(i)
                % infeasible starting value
                x0u(k) = -pi/2;
              elseif x0(i)>=UB(i)
                % infeasible starting value
                x0u(k) = pi/2;
              else
                x0u(k) = 2*(x0(i) - LB(i))/(UB(i)-LB(i)) - 1;
                % shift by 2*pi to avoid problems at zero in fminsearch
                % otherwise, the initial simplex is vanishingly small
                x0u(k) = 2*pi+asin(max(-1,min(1,x0u(k))));
              end

              % increment k
              k=k+1;
            case 0
              % unconstrained variable. x0u(i) is set.
              x0u(k) = x0(i);

              % increment k
              k=k+1;
            case 4
              % fixed variable. drop it before fminsearch sees it.
              % k is not incremented for this variable.
          end

        end

        % if any of the unknowns were fixed, then we need to shorten
        % x0u now.
        if k<=n
          x0u(k:n) = [];
        end

        % were all the variables fixed?
        if isempty(x0u)
          % All variables were fixed. quit immediately, setting the
          % appropriate parameters, then return.

          % undo the variable transformations into the original space
          x = obj.xtransform(x0u,params);

          % final reshape
          x = reshape(x,xsize);

          % stuff fval with the final value
          fval = feval(params.fun,x,params.args{:});

          % simplexSearchNMbnd was not called
          exitflag = 0;

          output.iterations = 0;
          output.funcount = 1;
          output.algorithm = 'fminsearch';
          output.message = 'All variables were held fixed by the applied bounds';

          % return with no call at all to fminsearch
          return
        end

        % Check for an outputfcn. If there is any, then substitute my
        % own wrapper function.
        if ~isempty(options.OutputFcn)
          params.OutputFcn = options.OutputFcn;
          options.OutputFcn = @outfun_wrapper;
        end

        % now we can call fminsearch, but with our own
        % intra-objective function.
        [xu,fval,exitflag,output] = fminsearch(@obj.intrafun,x0u,options,params);


        % undo the variable transformations into the original space
        x = obj.xtransform(xu,params);

        % final reshape
        x = reshape(x,xsize);

        % Use a nested function as the OutputFcn wrapper
          function stop = outfun_wrapper(x,varargin);
            % we need to transform x first
            xtrans = obj.xtransform(x,params);

            % then call the user supplied OutputFcn
            stop = params.OutputFcn(xtrans,varargin{1:(end-1)});

          end

        end % simplex algotithm implementation end


        function f = objFun(obj, X, opt_in)

            %   Objective function for optimization. Updates command
            %   timeseries in opt_in Simulink.SimulationInput object, runs
            %   the simulation with these commands, parses output and
            %   calculates objective function value based on energy storage
            %   and geostationary constraint satisfaction. This objective
            %   function uses quadratic penalties to enforce bounds on
            %   radius limit and altitude floor/ceiling, meaning that the
            %   constraitns may not be "strictly" satisfied.

            % for shorthand - number of time intervals
            n = obj.opt_params.n;
            %% make the changes here to enable optimization of only 2 states
            % add initial conditions and timings to command vectors
            t_evals      = linspace(0, obj.time.tfin, n+1);
            alt_rt_evals = [obj.init_conds.alt0_rate_init, X(1:n)];

            % set the initial values of optimization variables
            if (obj.opt_params.optVeas == true)
                veas_evals   = [obj.init_conds.veas_init,      X(n+1:2*n)] ;
                phi_evals    = [obj.init_conds.phi_0,          X(2*n+1:end)] ;
            else
                veas_evals   =  obj.init_conds.veas_init * ones(1,n+1) ;
                phi_evals    = [obj.init_conds.phi_0,          X(n+1:end)] ;
            end

            % convert vectors into timeseries (linearly interpolating too)
            alt_rt_cmd = timeseries(alt_rt_evals, t_evals);
            eas_cmd    = timeseries(veas_evals,   t_evals);
            phi_cmd    = timeseries(phi_evals,    t_evals);

            % add the commands as variables to the SimIn object
            oi = obj.setCmds(opt_in, alt_rt_cmd, eas_cmd, phi_cmd);

            % run the simulation
            iter_out = sim(oi);

            % get signals from Simulink.SimulationOutput object
            y = iter_out.get('yout');
            pm = iter_out.get('pwr_mdl_out');

            posn_north  = y.signals.values(:,10);  % north pos from origin (m)
            posn_east   = y.signals.values(:,11);  % east pos from origin (m)
            alt         = y.signals.values(:,12);  % altitude (m)
            v_tas       = y.signals.values(:,14);  % v_tas (m/s)
            E_batt      = pm.signals.values(:,9);  % energy in battery (kWh)
            E_gravit    = pm.signals.values(:,10); % gravitational energy (kWh)

            E_kinetic   = 0.5 * obj.vehicle.M * (v_tas.^2)/(1000*60*60);  % kinetic energy (kWh)

            %Get Sun's information
            sun_az      = pm.signals.values(:,1);        %sun azimuth signal (deg)
            sun_zth     = pm.signals.values(:,2);        %sun zenith signal (deg)

            %Get Aircraft's attitude information
            phi     = y.signals.values(:,7);   % bank attitude (rad)
            theta   = y.signals.values(:,8);    % pitch attitude (rad)
            psi     = y.signals.values(:,9);    % yaw attitude (rad)

            chi     = y.signals.values(:,21);   % coarse attitude (rad)

            psi_bounded     = mod(psi,(2*pi));   % yaw attitude (rad) within [0,2*pi]


            % net energy stored (gravitational + battery) from t_i to t_end
            if (obj.opt_params.opt_cost == 0 )

                E_stored = E_batt(end) + obj.opt_params.alt_weight*E_gravit(end) - E_batt(1) - obj.opt_params.alt_weight*E_gravit(1);

            elseif ( obj.opt_params.opt_cost == 1 )

                E_batt_ts        = timeseries(E_batt, obj.sim_out_parsed.t);        %time-series of the E_batt signal
                E_gravit_ts      = timeseries(E_gravit, obj.sim_out_parsed.t);      %time-series of the E_gravit signal
                E_kinetic_ts     = timeseries(E_kinetic, obj.sim_out_parsed.t);     %time-series of the E_kinetic signal

                E_batt_ds      = resample(E_batt_ts, t_evals);                      %down-sampled (decimated) E_batt signal
                E_gravit_ds = resample(E_gravit_ts, t_evals);                 %down-sampled (decimated) E_gravit signal
                E_kinetic_ds   = resample(E_kinetic_ts, t_evals);                   %down-sampled (decimated) E_gravit signal

                E_stored_ds    = E_batt_ds.Data + E_gravit_ds.Data + E_kinetic_ds.Data ;
                E_stored = 0 ;

                for i=1:(size(E_batt_ds.Data,1)-1)

                    E_stored = E_stored + ( sign(E_stored_ds(i+1) - E_stored_ds(i)) * (E_stored_ds(i+1) - E_stored_ds(i))^2 ) ;

                end


            elseif ( obj.opt_params.opt_cost == 2 )

                E_stored   =    E_batt(end) + obj.opt_params.alt_weight*E_gravit(end) - E_batt(1) - obj.opt_params.alt_weight*E_gravit(1) ;
                for jj=1:size(phi,1)

                    Cn2b              = EnergySim.NED2BdyTransMatix(phi(jj),theta(jj),psi_bounded(jj)) ;
                    %calculate Sun's normal vector in the NED frame
                    sun_vec_ned       = [sind(sun_zth(jj))*cosd(sun_az(jj)) ;  sind(sun_zth(jj))*sind(sun_az(jj)) ;  -cosd(sun_zth(jj))];
                    sun_vec_bdy       = Cn2b*sun_vec_ned ;

                    Intercept_angle(jj)    = acos(dot(sun_vec_bdy,[0 0 -1]));   %in rad  in [0,pi] - should put a function to check the bounds
                end

                Mean_intercept_angle   =  mean(Intercept_angle) ;  %in rad

            end


            % conversion from kWh to kW (average over simulation)
            P_avg_stored = E_stored*60*60/obj.time.tfin;

            % initialize altitude and position constraint penalties to zero
            alt_penalty = 0;
            posn_penalty = 0;

            % calculate penalty for going below altitude floor
            if min(alt) < obj.vehicle.alt_lim_l+30
                alt_penalty = 16000 * ((min(alt) - (obj.vehicle.alt_lim_l+30) )/obj.vehicle.alt_lim_l)^2;
            end

            % calculate penalty for exceeding altitude ceiling
            if max(alt) > obj.vehicle.alt_lim_h-30
                alt_penalty = 16000 * ((max(alt) - (obj.vehicle.alt_lim_h-30) )/obj.vehicle.alt_lim_h)^2;
            end

            % radius (from point [0, 0])
            R = (posn_north.^2 + posn_east.^2).^0.5;

            % calculate penalty for exceeding 3km radius from IC
            if max(R) > 3000
                posn_penalty = 10 * ((max(R) - 3000)/3000)^2;
            end

            % return objective function value
            f = -P_avg_stored + alt_penalty + posn_penalty;

        end

    end  %%end of private methods

    methods (Static, Access = 'private')

        function [ICs, fp_courses, fp_coords] = msnInit(fp, init_wp, x_0, y_0)
            % Defaulting to the first WP in the sequence.
            to_init = fp(init_wp, 1);
            next_init = fp(init_wp, 3);

            % Parse mission to compute chi, next chi,
            % delta_chi, entry_x, entry_y

            [fp_courses,fp_coords] = msn_params(fp);

            to_x = fp(to_init,4);
            to_y = fp(to_init,5);

            next_x = fp(next_init,4);
            next_y = fp(next_init,5);
            dx = to_x - x_0;
            dy = to_y - y_0;
            dchi0 = atan2(next_y - to_y, next_x - to_x) - atan2(dy, dx);
            dchi0 = atan2(sin(dchi0), cos(dchi0));
            d_init = sqrt(dx*dx + dy*dy);

            ICs = [to_init, next_init, d_init, dchi0];

        end

        function [absorb_h, alpha_h] = atmosAbsorb()
            % [absorb_h, alpha_h] = atmosAbsorb()
            %   Implementation of absorb2.m (originally by Bruce Lin,
            %   2015). Used in Simulink for the computation of atmospheric
            %   absorption of solar power.

            % define P0 at h=0. Throw away all other atmospheric properties.
            [~,~,~,P0,~,~] = EnergySim.stdatmo(0);

            % height in m
            absorb_h = 0:100:40000;

            % absorbance ratio dI/I, as a function of h. Dummy to dimension
            alpha_h = absorb_h;

            % populate the pressure array in Pa
            [~,~,~,P,~,~] = EnergySim.stdatmo(absorb_h);

            % Calculate array of intensities (rel. to max) as a function of P/P0
            I_rel = 1.0 * exp(-P/P0 * 0.32);
            for i = 1:length(absorb_h)-1
                % absorbance is a ratio of intensities
                alpha_h(i) = -log(I_rel(i+1)/I_rel(i)) / (absorb_h(i+1)-absorb_h(i)) ;
            end

            % force last value to be repeated. absorbance is so small here (alpha close to 0) that it doesn't make a difference
            alpha_h(i+1)=alpha_h(i);
        end

        function aero = setAeroData(vehicle)
            % aero = setAeroData(vehicle)
            %   Set the aerodynamic data lookup (or reverse lookup) tables
            %   used in simulation and also trimming aircraft ICs.
            %   Additional configurations can be added.

            % check configuration name from vehicle struct
            if strcmp(vehicle.configuration,'AF_BASELINE')

                % altitudes
                aero.alt_bp = [0 20574 21908 23241 24575];

                % AoAs
                aero.alpha_bp = (-4.5:0.5:4.5) * EnergySim.d2r;

                % lift coefficient vs. altitude, aoa
                aero.cl_data = [0.9105	0.8666	0.8383	0.7964	0.734
                                0.9717	0.929	0.8983	0.8573	0.7946
                                1.0303	0.9885	0.9581	0.9163	0.8541
                                1.087	1.0463	1.0171	0.9738	0.9135
                                1.1407	1.1018	1.074	1.0313	0.9701
                                1.1909	1.1526	1.1269	1.0873	1.0254
                                1.2367	1.1979	1.1743	1.1383	1.0787
                                1.2769	1.2368	1.2149	1.1814	1.1262
                                1.3107	1.2691	1.2491	1.2179	1.1663
                                1.3385	1.295	1.277	1.2483	1.1994
                                1.3608	1.3153	1.2993	1.2727	1.226
                                1.378	1.3304	1.3161	1.2914	1.2469
                                1.3914	1.3417	1.3291	1.3063	1.2639
                                1.4017	1.3503	1.3391	1.3182	1.278
                                1.41	1.3572	1.3471	1.3278	1.2898
                                1.4166	1.3629	1.3535	1.3356	1.2995
                                1.4223	1.3677	1.3591	1.3422	1.3077
                                1.4273	1.3719	1.3645	1.3481	1.3147
                                1.4317	1.3757	1.3694	1.3534	1.3207];

                % lift coefficient vs. altitude, aoa
                aero.cd_data = [0.016	0.0246	0.0285	0.0333	0.0392
                                0.017	0.0254	0.0293	0.034	0.0402
                                0.0181	0.0261	0.0299	0.0348	0.0411
                                0.0193	0.0269	0.0307	0.0356	0.0419
                                0.0206	0.0277	0.0314	0.0363	0.0426
                                0.0219	0.0287	0.0321	0.0368	0.0432
                                0.0232	0.0297	0.033	0.0373	0.0437
                                0.0244	0.0307	0.0338	0.038	0.0441
                                0.0256	0.0318	0.0347	0.0387	0.0446
                                0.0267	0.0328	0.0356	0.0395	0.0451
                                0.0276	0.0337	0.0364	0.0402	0.0457
                                0.0284	0.0345	0.0372	0.0409	0.0463
                                0.0291	0.0353	0.0379	0.0415	0.0468
                                0.0296	0.0359	0.0385	0.0421	0.0472
                                0.0302	0.0365	0.0391	0.0426	0.0477
                                0.0307	0.037	0.0396	0.043	0.0481
                                0.0311	0.0375	0.04	0.0435	0.0485
                                0.0315	0.0379	0.0405	0.0439	0.0489
                                0.0319	0.0383	0.0409	0.0443	0.0493];

                aero.max_cl = max(aero.cl_data(:));
                aero.min_cl = min(aero.cl_data(:));
                aero.max_alt_bp = max(aero.alt_bp);
                aero.min_alt_bp = min(aero.alt_bp);
                aero.max_alpha_bp = max(aero.alpha_bp);
                aero.min_alpha_bp = min(aero.alpha_bp);

                [aero.cl_bp, aero.aoa_data] = EnergySim.aoa_rlut_tbl(aero.cl_data, aero.alt_bp, aero.alpha_bp);
            else
                error('Error. Only AF_BASELINE aero data available.');
            end

        end

        %To calculate the transformation matrix from NED frame to Body
        %frame - angles inputs are in rad (p:Phi, t:Theta, s:Psi)
        function Cnb = NED2BdyTransMatix(p,t,s)

            Cnb(1,1) = cos(t)*cos(s) ;
            Cnb(1,2) = cos(t)*sin(s) ;
            Cnb(1,3) = -sin(t) ;
            Cnb(2,1) = -cos(p)*sin(s) + sin(p)*sin(t)*cos(s) ;
            Cnb(2,2) = cos(p)*cos(s) + sin(p)*sin(t)*sin(s) ;
            Cnb(2,3) = sin(p)*cos(t) ;
            Cnb(3,1) = sin(p)*sin(s) + cos(p)*sin(t)*cos(s) ;
            Cnb(3,2) = -sin(p)*cos(s) + cos(p)*sin(t)*sin(s) ;
            Cnb(3,3) = cos(p)*cos(t) ;
        end

        function wind_struct = testWind(wind, testStation, tst, alt)
            %   Fill in anything missing from supplied wind struct. This
            %   gives the option of either supplying magnitude and
            %   direction or NED components.

            assert(exist('wind','var')==1, 'Please supply a wind struct to create an EnergySim object');
            wind_struct.Wd = wind.Wd;

            wmag_level_idx = interp1(testStation.percs,[1:1:7], tst.wmag_level,'nearest');
            Wmag_alt = reshape(testStation.wnd_mag(tst.dayIdx, wmag_level_idx, :),[1 6]);
            wind_struct.Wmag = interp1(testStation.alts, Wmag_alt, alt, 'linear');

            wind_struct.Wdir = genTestWindDir(testStation, tst, alt);

            wind_struct.Wn = wind.Wmag * cos(wind.Wdir);
            wind_struct.We = wind.Wmag * sin(wind.Wdir);

        end

        function wind_struct = setWind(wind)
            % wind_struct = setWind(wind)
            %   Fill in anything missing from supplied wind struct. This
            %   gives the option of either supplying magnitude and
            %   direction or NED components.

            assert(exist('wind','var')==1, 'Please supply a wind struct to create an EnergySim object');

            % if magnitude and direction are supplied, overwrite NED components
            if(isfield(wind, 'Wdir') && isfield(wind, 'Wmag'))
                wind_struct.Wdir = wind.Wdir;
                wind_struct.Wmag = wind.Wmag;
                wind_struct.Wn = wind.Wmag * cos(wind.Wdir);
                wind_struct.We = wind.Wmag * sin(wind.Wdir);
                wind_struct.Wd = 0;
            else
                % otherwise overwrite magnitude and direction
                wind_struct.Wn = wind.Wn;
                wind_struct.We = wind.We;
                wind_struct.Wd = wind.Wd;
                wind_struct.Wmag = sqrt([wind.Wn wind.We wind.Wd] * [wind.Wn wind.We wind.Wd]');
                wind_struct.Wdir = atan2(wind.We, wind.Wn);
            end

        end

        function vehicle_struct = setVehicle(vehicle)
            % vehicle_struct = setVehicle(vehicle)
            %   Fill in remainder of vehicle struct.

            assert(exist('vehicle','var')==1, 'Please supply a vehicle struct to create an EnergySim object');
            vehicle_struct = vehicle;

            % multiply unit normals by areas, if not already done
            if(~isfield(vehicle, 'panel_vecs'))
                vehicle_struct.panel_vecs = (vehicle.panel_dirs.*vehicle.panel_areas)';
            end

            % aspect ratio
            if(~isfield(vehicle, 'AR'))
                vehicle_struct.AR = (vehicle.b^2) / vehicle.S;
            end

        end

        function time_struct = setTime(time)
            % time_struct = setTime(time)
            %   Fill in remainder of vehicle struct. This includes
            %   calculation of simulation time in seconds, and start time
            %   in Unix time (seconds).

            time_init = struct('year', time.year, 'month', time.month, ...
                'day', time.day, 'hour', time.hour, 'min', time.min, ...
                'UTC', time.UTC);
            
            % MATLAB datetime objects for start and finish
            dt_init = datetime(time.year, time.month, time.day, time.hour, time.min, 0);
            
            % Finish time is offset from starting time
            dt_fin = dt_init + hours(time.run.hours) + minutes(time.run.mins);

            % Unix time for start
            t_init = posixtime(dt_init);

            % simulation time (seconds)
            tfin = posixtime(dt_fin)-posixtime(dt_init);
            assert(tfin > 0, 'Invalid time period specified for simulation');
            time_struct = struct('tfin', tfin, 't_init', t_init, ...
                'UTC', time.UTC, 'time_init', time_init);
        end

        function ic_struct = setICs(init_conds)
            % ic_struct = setICs(init_conds)
            %   Fill in remainder of initial conditions struct (unused for
            %   now).

            assert(exist('init_conds','var')==1, 'Please supply an initial conditions struct to create an EnergySim object');
            ic_struct = init_conds;

        end

        function par_struct = setParParams(pars)
            % par_struct = setParParams(pars)
            %   Fill in remainder of parameters for parametric (i.e. rule
            %   based) simulations

            assert(exist('pars','var')==1, 'Please supply a parametric sim struct to create an EnergySim object');
            assert(ismember(pars.chi_cmd_mode, [1,2,3,4,5]), 'Parametric course command mode not valid.');
            assert(ismember(pars.alt_cmd_mode, [1,2]),   'Parametric altitude command mode not valid.');

            if pars.alt_cmd_mode == 2
                assert(pars.chi_cmd_mode == 2, 'High-frequency climb/glide only works with racetrack course command mode.');
            end

            par_struct = pars;

        end

        function opt_struct = setOptParams(pars)
            % opt_struct = setOptParams(pars)
            %   Fill in remainder of parameters for optimizations (unused
            %   for now).

            opt_struct = pars;

            % input validation for Fourier Transformation
            if isfield(pars, 'opt_opt') && pars.opt_opt == 2
                if min(pars.fft_order) <= 0
                    error('Fourier Transformation is enabled, but order of coefficients is not corrcetly provided because some component is not positive.')
                end
            end

        end

        function si_ret = setSigs(sim_in)
            % si_ret = setSigs(sim_in)
            %   Update sim_in (Simulink.SimulationInput object) with
            %   root-level signal data (needed for autocoding)

            % input and output signals for autocoding
            si = sim_in;
            si = si.setVariable('K6DOF_TYPE', EnergySim.CreateSlkObj('DataType', 'Double'));
            si = si.setVariable('u_vert_cmd', EnergySim.CreateSlkObj('Signal', 'Auto', 'K6DOF_TYPE', 1));
            si = si.setVariable('u_eas_cmd', EnergySim.CreateSlkObj('Signal', 'Auto', 'K6DOF_TYPE', 1));
            si = si.setVariable('u_lat_cmd', EnergySim.CreateSlkObj('Signal', 'Auto', 'K6DOF_TYPE', 1));
            si = si.setVariable('u_winds', EnergySim.CreateSlkObj('Signal', 'Auto', 'K6DOF_TYPE', 3));
            si = si.setVariable('av_state', EnergySim.CreateSlkObj('Signal', 'Auto', 'K6DOF_TYPE', 28));
            si = si.setVariable('pwr_state', EnergySim.CreateSlkObj('Signal', 'Auto', 'K6DOF_TYPE', 18));

            % return updated Simulink.SimulationInput object
            si_ret = si;

        end

        % the following functions are copied from individual files
        function [cl_bp, aoa_data] = aoa_rlut_tbl(cl_data, alt_bp, alpha_bp)
            % [cl_bp, aoa_data] = aoa_rlut_tbl(cl_data, alt_bp, alpha_bp)
            %   Angle of attack reverse lookup table (Jack Marriott)

            max_cl = max(cl_data(:));
            min_cl = min(cl_data(:));
            n_cl = 20;
            cl_bp = linspace(min_cl, max_cl,n_cl);
            n = length(alt_bp);
            aoa_data = zeros(n_cl, n);

            for i = 1:n
                aoa_ip = interp1(cl_data(:,i),alpha_bp,cl_bp,'linear');
                aoa_data(:,i)  = aoa_ip';
            end

            % TODO: At the present time, these limits only work with
            %       AF_BASELINE. Need to update for the other airframes.

            for i = 1:n
                for j = floor(n_cl/2):-1:1
                    if (isnan(aoa_data(j,i)))
                        aoa_data(j,i) = -4.5 * pi / 180.0;
                    end
                end
                for j = floor(n_cl/2)+1:n_cl
                    if (isnan(aoa_data(j,i)))
                        aoa_data(j,i) = 4.5 * pi / 180.0;
                    end
                end
            end
        end

        function [vtas_init, vgnd_init, ga_init] = init_veloc(rho_sl, veas_init, chi_init, W_vect, rho_alt)
            %INIT_VELOC Compute initial velocities and FPA

            vtas_init = veas_init * sqrt(rho_sl/rho_alt);
            ga_init = asin(W_vect(3) / vtas_init);

            W = sqrt(W_vect * W_vect');
            psi = chi_init - asin(W_vect(1:2) * [-sind(chi_init);cosd(chi_init)] /...
                vtas_init / cosd(ga_init)) * 180 / pi;
            Va_vect = vtas_init * [cosd(psi)*cosd(ga_init);sind(psi)*cosd(ga_init);-sind(ga_init)]';

            Vg = vtas_init * vtas_init + W * W + 2 * Va_vect * W_vect';
            vgnd_init = sqrt(Vg);
        end

        function alpha_est = init_alpha(cl0, alt_init, cl_data, alt_bp, alpha_bp)
            %INIT_ALPHA Compute alpha given CL

            lb = find(alt_bp <= alt_init);
            if ~isempty(lb)
                lb = lb(end);
            end

            [~,c] = size(cl_data);

            if (lb+1 > c)
                alpha_est = interp1(cl_data(:,lb),alpha_bp,cl0);
            elseif (isempty(lb))
                alpha_est = interp1(cl_data(:,1),alpha_bp,cl0);
            else
               alpha_lo = interp1(cl_data(:,lb),alpha_bp,cl0);
                alpha_hi = interp1(cl_data(:,lb+1),alpha_bp,cl0);
                w = (alt_init - alt_bp(lb))/(alt_bp(lb+1) - alt_bp(lb));
                alpha_est = (1 - w) * alpha_lo + w * alpha_hi;
            end

        end

        function [rho,a,temp,press,kvisc,ZorH] = stdatmo(H_in,Toffset,Units,GeomFlag)
            %  STDATMO Find gas properties in earth's atmosphere.
            %   [rho,a,T,P,nu,ZorH] = STDATMO(H,dT,Units,GeomFlag)
            %
            %   STDATMO by itself gives the atmospheric properties at sea level on a
            %   standard day.
            %
            %   STDATMO(H) returns the properties of the 1976 Standard Atmosphere at
            %   geopotential altitude H (meters), where H is a scalar, vector, matrix,
            %   or ND array.
            %
            %   STDATMO(H,dT) returns properties when the temperature is dT degrees
            %   offset from standand conditions. H and dT must be the same size or else
            %   one must be a scalar.
            %
            %   STDATMO(H,dT,Units) specifies units for the inputs outputs. Options are
            %   SI (default) or US (a.k.a. Imperial, English). For SI, set Units to []
            %   or 'SI'. For US, set Units to 'US'. Input and output units may be
            %   different by passing a cell array of the form {Units_in Units_out},
            %   e.g. {'US' 'SI'}. Keep in mind that dT is an offset, so when converting
            %   between Celsius and Fahrenheit, use only the scaling factor (dC/dF =
            %   dK/dR = 5/9). Units are as follows:
            %       Input:                        SI (default)    US
            %           H:     Altitude           m               ft
            %           dT:    Temp. offset       °C/°K           °F/°R
            %       Output:
            %           rho:   Density            kg/m^3          slug/ft^3
            %           a:     Speed of sound     m/s             ft/s
            %           T:     Temperature        °K              °R
            %           P:     Pressure           Pa              lbf/ft^2
            %           nu:    Kinem. viscosity   m^2/s           ft^2/s
            %           ZorH:  Height or altitude m               ft
            %
            %   STDATMO(H,dT,u), where u is a structure created by the UNITS function,
            %   accepts variables of the DimensionedVariable class as inputs. Outputs
            %   are of the DimensionedVariable class. If a DimensionedVariable is not
            %   provided for an input, STDATMO assumes SI input.
            %
            %   STDATMO(H,dT,Units,GeomFlag) with logical input GeomFlag returns
            %   properties at geometric altitude input H instead of the normal
            %   geopotential altitude.
            %
            %   [rho,a,T,P,nu] = STDATMO(H,dT,...) returns atmospheric properties the
            %   same size as H and/or dT (P does not vary with temperature offset and
            %   is always the size of H)
            %
            %   [rho,a,T,P,nu,ZorH] = STDATMO(H,...) returns either geometric height,
            %   Z, (GeomFlag not set) or geopotential height, H, (GeomFlag set).
            %
            %   Example 1: Find atmospheric properties at every 100 m of geometric
            %   height for an off-standard atmosphere with temperature offset varying
            %   +/- 25°C sinusoidally with a period of 4 km.
            %       Z = 0:100:86000;
            %       [rho,a,T,P,nu,H] = stdatmo(Z,25*sin(pi*Z/2000),'',true);
            %       semilogx(rho/stdatmo,H/1000)
            %       title('Density variation with sinusoidal off-standard atmosphere')
            %       xlabel('\sigma'); ylabel('Altitude (km)')
            %
            %   Example 2: Create tables of atmospheric properties up to 30000 ft for a
            %   cold (-15°C), standard, and hot (+15°C) day with columns
            %   [h(ft) Z(ft) rho(slug/ft³) sigma a(ft/s) T(R) P(psf) µ(slug/ft-s) nu(ft²/s)]
            %   using 3-dimensional array inputs.
            %       [~,h,dT] = meshgrid(0,-5000:1000:30000,-15:15:15);
            %       [rho,a,T,P,nu,Z] = stdatmo(h,dT*9/5,'US',0);
            %       Table = [h Z rho rho/stdatmo(0,0,'US') T P nu.*rho nu];
            %       format short e
            %       ColdTable       = Table(:,:,1)
            %       StandardTable   = Table(:,:,2)
            %       HotTable        = Table(:,:,3)
            %
            %   Example 3: Use the unit consistency enforced by the DimensionedVariable
            %   class to find the SI dynamic pressure, Mach number, Reynolds number, and
            %   stagnation temperature of an aircraft flying at flight level FL500
            %   (50000 ft) with speed 500 knots and characteristic length of 80 inches.
            %       u = units;
            %       V = 500*u.kts; c = 80*u.in;
            %       [rho,a,T,P,nu] = stdatmo(50*u.kft,[],u);
            %       Dyn_Press = 1/2*rho*V^2;
            %       M = V/a;
            %       Re = V*c/nu;
            %       T0 = T*(1+(1.4-1)/2*M^2);
            %
            %   This atmospheric model is not recommended for use at altitudes above
            %   86 km geometric height (84852 m/278386 ft geopotential) and returns NaN
            %   for altitudes above 90 km geopotential.
            %
            %   See also ATMOSISA, ATMOSNONSTD,
            %     DENSITYALT - http://www.mathworks.com/matlabcentral/fileexchange/39325,
            %     UNITS      - http://www.mathworks.com/matlabcentral/fileexchange/38977.
            %
            %   [rho,a,T,P,nu,ZorH] = STDATMO(H,dT,Units,GeomFlag)

            %   Copyright 2010-2014 Sky Sartorius
            %   www.mathworks.com/matlabcentral/fileexchange/authors/101715
            %
            %   References: ESDU 77022;  www.pdas.com/atmos.html

            if nargin >= 3 && isstruct(Units)
                U = true;
                u = Units;
                if isa(H_in,'DimensionedVariable')
                    H_in = H_in/u.m;
                end
                if isa(Toffset,'DimensionedVariable')
                    Toffset = Toffset/u.K;
                end

                Units = 'si';
            else
                U = false;
            end

            if nargin == 0
                H_in = 0;
            end

            if nargin < 2 || isempty(Toffset)
                Toffset = 0;
            end

            if nargin <= 2 && all(H_in(:) <= 11000) %quick troposphere-only code
                TonTi=1-2.255769564462953e-005*H_in;
                press=101325*TonTi.^(5.255879812716677);
                temp = TonTi*288.15 + Toffset;
                rho = press./temp/287.05287;

                if nargout > 1
                    a = sqrt(401.874018 * temp);
                    if nargout >= 5
                        kvisc = (1.458e-6 * temp.^1.5 ./ (temp + 110.4)) ./ rho;
                        if nargout == 6 % Assume Geop in, find Z
                            ZorH = 6356766*H_in./(6356766-H_in);
                        end
                    end
                end
                return
            end

            % index   Lapse rate   Base Temp     Base Geopo Alt        Base Pressure
            %   i      Ki (°C/m)    Ti (°K)         Hi (m)              P (Pa)
            D =[1       -.0065      288.15          0                   101325
                2       0           216.65          11000               22632.0400950078
                3       .001        216.65          20000               5474.87742428105
                4       .0028       228.65          32000               868.015776620216
                5       0           270.65          47000               110.90577336731
                6       -.0028      270.65          51000               66.9385281211797
                7       -.002       214.65          71000               3.9563921603966
                8       0           186.94590831019 84852.0458449057    0.373377173762337  ];

            % Constants
            R=287.05287;	%N-m/kg-K; value from ESDU 77022
            % R=287.0531;   %N-m/kg-K; value used by MATLAB aerospace toolbox ATMOSISA
            gamma=1.4;
            g0=9.80665;     %m/sec^2
            RE=6356766;     %Radius of the Earth, m
            Bs = 1.458e-6;  %N-s/m2 K1/2
            S = 110.4;      %K

            K=D(:,2);	%°K/m
            T=D(:,3);	%°K
            H=D(:,4);	%m
            P=D(:,5);	%Pa

            temp=zeros(size(H_in));
            press=temp;
            hmax = 90000;

            if nargin < 3 || isempty(Units)
                Uin = false;
                Uout = Uin;
            elseif isnumeric(Units) || islogical(Units)
                Uin = Units;
                Uout = Uin;
            else
                if ischar(Units) %input and output units the same
                    Unitsin = Units; Unitsout = Unitsin;
                elseif iscell(Units) && length(Units) == 2
                    Unitsin = Units{1}; Unitsout = Units{2};
                elseif iscell(Units) && length(Units) == 1
                    Unitsin = Units{1}; Unitsout = Unitsin;
                else
                    error('Incorrect Units definition. Units must be ''SI'', ''US'', or 2-element cell array')
                end

                if strcmpi(Unitsin,'si')
                    Uin = false;
                elseif strcmpi(Unitsin,'us')
                    Uin = true;
                else error('Units must be ''SI'' or ''US''')
                end

                if strcmpi(Unitsout,'si')
                    Uout = false;
                elseif strcmpi(Unitsout,'us')
                    Uout = true;
                else error('Units must be ''SI'' or ''US''')
                end
            end

            % Convert from imperial units, if necessary.
            if Uin
                H_in = H_in * 0.3048;
                Toffset = Toffset * 5/9;
            end



            % Convert from geometric altitude to geopotental altitude, if necessary.
            if nargin < 4
                GeomFlag = false;
            end
            if GeomFlag
                Hgeop=(RE*H_in)./(RE+H_in);
            else
                Hgeop=H_in;
            end

            n1=(Hgeop<=H(2));
            n2=(Hgeop<=H(3) & Hgeop>H(2));
            n3=(Hgeop<=H(4) & Hgeop>H(3));
            n4=(Hgeop<=H(5) & Hgeop>H(4));
            n5=(Hgeop<=H(6) & Hgeop>H(5));
            n6=(Hgeop<=H(7) & Hgeop>H(6));
            n7=(Hgeop<=H(8) & Hgeop>H(7));
            n8=(Hgeop<=hmax & Hgeop>H(8));
            n9=(Hgeop>hmax);

            % Troposphere
            if any(n1(:))
                i=1;
                TonTi=1+K(i)*(Hgeop(n1)-H(i))/T(i);
                temp(n1)=TonTi*T(i);
                PonPi=TonTi.^(-g0/(K(i)*R));
                press(n1)=P(i)*PonPi;
            end

            % Tropopause
            if any(n2(:))
                i=2;
                temp(n2)=T(i);
                PonPi=exp(-g0*(Hgeop(n2)-H(i))/(T(i)*R));
                press(n2)=P(i)*PonPi;
            end

            % Stratosphere 1
            if any(n3(:))
                i=3;
                TonTi=1+K(i)*(Hgeop(n3)-H(i))/T(i);
                temp(n3)=TonTi*T(i);
                PonPi=TonTi.^(-g0/(K(i)*R));
                press(n3)=P(i)*PonPi;
            end

            % Stratosphere 2
            if any(n4(:))
                i=4;
                TonTi=1+K(i)*(Hgeop(n4)-H(i))/T(i);
                temp(n4)=TonTi*T(i);
                PonPi=TonTi.^(-g0/(K(i)*R));
                press(n4)=P(i)*PonPi;
            end

            % Stratopause
            if any(n5(:))
                i=5;
                temp(n5)=T(i);
                PonPi=exp(-g0*(Hgeop(n5)-H(i))/(T(i)*R));
                press(n5)=P(i)*PonPi;
            end

            % Mesosphere 1
            if any(n6(:))
                i=6;
                TonTi=1+K(i)*(Hgeop(n6)-H(i))/T(i);
                temp(n6)=TonTi*T(i);
                PonPi=TonTi.^(-g0/(K(i)*R));
                press(n6)=P(i)*PonPi;
            end

            % Mesosphere 2
            if any(n7(:))
                i=7;
                TonTi=1+K(i)*(Hgeop(n7)-H(i))/T(i);
                temp(n7)=TonTi*T(i);
                PonPi=TonTi.^(-g0/(K(i)*R));
                press(n7)=P(i)*PonPi;
            end

            % Mesopause
            if any(n8(:))
                i=8;
                temp(n8)=T(i);
                PonPi=exp(-g0*(Hgeop(n8)-H(i))/(T(i)*R));
                press(n8)=P(i)*PonPi;
            end

            if any(n9(:))
                warning('One or more altitudes above upper limit.')
                temp(n9)=NaN;
                press(n9)=NaN;
            end

            temp = temp + Toffset;

            rho = press./temp/R;

            if nargout >= 2
                a = sqrt(gamma * R * temp);
                if nargout >= 5
                    kvisc = (Bs * temp.^1.5 ./ (temp + S)) ./ rho; %m2/s
                    if nargout == 6
                        if GeomFlag % Geometric in, ZorH is geopotential altitude (H)
                            ZorH = Hgeop;
                        else % Geop in, find Z
                            ZorH = RE*Hgeop./(RE-Hgeop);
                        end
                    end
                end
            end

            if Uout %convert to imperial units if output in imperial units
                rho = rho / 515.3788;
                if nargout >= 2
                    a = a / 0.3048;
                    temp = temp * 1.8;
                    press = press / 47.88026;
                    if nargout >= 5
                        kvisc = kvisc / 0.09290304;
                        if nargout == 6
                            ZorH = ZorH / 0.3048;
                        end
                    end
                end
            end

            if U
                rho = rho*u.kg/(u.m^3);
                if nargout >= 2
                    a = a*u.m/u.s;
                    temp = temp*u.K;
                    press = press*u.Pa;
                    if nargout >= 5
                        kvisc = kvisc*u.m^2/u.s;
                        if nargout == 6
                            ZorH = ZorH*u.m;
                        end
                    end
                end
            end
            % Credit for elements of coding scheme:
            % cobweb.ecn.purdue.edu/~andrisan/Courses/AAE490A_S2001/Exp1/

            % TODO: allow single input of DimensionedVariable, with warning on speed.

            % Revision history:
            %{
            V1.0    5 July 2010
            V1.1    8 July 2010
                Update to references and improved input handling
            V2.0    12 July 2010
                Changed input ImperialFlag to Units. Units must now be a string or cell
                array {Units_in Units_out}. Version 1 syntax works as before.
              Two examples added to help
            V2.1    15 July 2010
                Changed help formatting
                Sped up code - no longer caclulates a or nu if outputs not specified.
                Also used profiler to speed test against ATMOSISA, which is
                consistently about 5 times slower than STDATMO
                      17 July 2010
                Cleaned up Example 1 setup using meshgrid
                      26 July 2010
                Switched to logical indexing, which sped up running Example 1
                significantly(running [rho,a,T,P,nu,h]=stdatmo(Z,dT,'US',1) 1000 times:
                ~.67s before, ~.51s after)
            V3.0    7 August 2010
                Consolodated some lines for succintness Changed Hgeop output to be
                either geopotential altitude or geometric altitude, depending on which
                was input. Updated help and examples accordingly.
            V3.1    27 August 2010
                Added a very quick, troposhere-only section
            V3.2    23 December 2010
                Minor changes, tested on R2010a, and sinusoidal example added
            V4.0    6 July 2011
                Imperial temp offset now °F/°R instead of °C/°K
            V4.1    12 Sep 2012
                Added ZorH output support for quick troposphere calculation
                uploaded
            V4.2
              tiny changes to help and input handling
                nov 2012: some :s added to make use of any() better
                added see alsos
              uploaded
            V5.0
              STDATMODIM wrapper created that takes DimensionedVariable input
              uploaded 5 Dec 2012
            V6.0
              STDATMODIM functionality integrated into STDATMO; example three changed
              for illustration.
            2013-05-07 comment block changes
            2014-05-18 minor formatting
            %}
        end

        function slk_obj = CreateSlkObj(dClass, varargin)
            % Create a data object and add it as Simulink variable

            % Special handling for different classes of data.
            switch dClass
             case 'DataType'
              narginchk(2,2);
              tmpObj = Simulink.NumericType;
              % Set DataTypeMode
              tmpObj.DataTypeMode = varargin{1};
              tmpObj.IsAlias = true;

             case 'Parameter'
              narginchk(4,4);
              tmpObj = Simulink.Parameter;
              % Set Value
              sc = varargin{1};
              tmpObj.Value = varargin{2};
              % Set StructName for Custom storage class
              structName = 'PARAM';

             case {'Signal', 'State'}
              narginchk(4,4);
              tmpObj = Simulink.Signal;
              % Set state attributes
              sc = varargin{1};
              tmpObj.DataType = varargin{2};
              tmpObj.Dimensions = varargin{3};
              tmpObj.Complexity = 'real';
              tmpObj.SampleTime = -1;

              % Set StructName for Custom storage class
              if isequal(dClass, 'Signal')
                structName = 'SIGNAL';
              else
                structName = 'STATE';
              end
             otherwise
              DAStudio.error('rtwdemos:rtwdemo_advsc_data:InvalidClass')
            end

            if exist('sc', 'var')
              % Set common data attributes:
              % - StorageClass
              tmpObj.CoderInfo.StorageClass = sc;

              % - CustomStorageClass
              if strcmp(sc, 'Custom')
                tmpObj.CoderInfo.CustomStorageClass = 'ExportToFile';
            %    tmpObj.CoderInfo.CustomAttributes.StructName = structName;
                tmpObj.CoderInfo.CustomAttributes.HeaderFile = strcat(varargin{3},'.h');
                tmpObj.CoderInfo.CustomAttributes.DefinitionFile = strcat(varargin{3},'.c');
              end
            end

            slk_obj = tmpObj;
        end

    end

end
