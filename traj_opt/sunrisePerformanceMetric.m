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
function [delta_soc, delta_t] = sunrisePerformanceMetric(ES, sun_time_before, sun_time_after, energy_type)
    % plot performance metric graph and calculate delta SOC and delta t
    % Assumption: the ES file is NOT using autowinds, a season is picked
    
    tst_station = ES.testStation.tst_station;
    tst_option  = ES.testOptions;
    
    opt = ES.opt_out_parsed;
    if(isempty(opt))
        error('Error: It appears that optimization has not been run/parsed yet');
    end
    baseline = ES.sim_out_parsed;
    if(isempty(baseline))
        error('Error: It appears that simulation has not been run/parsed yet');
    end
    
    % plot Battery Energy change over time
    figure;
    xlim([0 opt.t(end)/(60*60)]);
    
    
    if strcmp(energy_type, 'battery')
        plot(opt.t_hr, opt.E_batt);
        grid on; hold on;
        plot(baseline.t_hr, baseline.E_batt);
        ylabel('Battery Energy (kWh)');
    elseif strcmp(energy_type, 'potential')
        plot(opt.t_hr, opt.E_gravit);
        grid on; hold on;
        plot(baseline.t_hr, baseline.E_gravit);
        ylabel('Potential Energy (kWh)');
    elseif strcmp(energy_type, 'total')
        plot(opt.t_hr, opt.E_batt + opt.E_gravit);
        grid on; hold on;
        plot(baseline.t_hr, baseline.E_batt + baseline.E_gravit);
        ylabel('Total Energy (kWh)');
    else
        error('Energy type input invalid!')
    end
    
    % plot the difference of SOC between optimization and baseline
    [opt_batt_min, opt_idx] = min(opt.E_batt);
    [baseline_batt_min, baseline_idx] = min(baseline.E_batt);
    plot(opt.t_hr, opt_batt_min * ones(length(opt.E_batt), 1), 'k--');
    plot(opt.t_hr, baseline_batt_min * ones(length(baseline.E_batt), 1), 'k--');

    
    % calcualte key performance metrics
    delta_soc = opt_batt_min - baseline_batt_min;                 % kwh
    delta_t = (opt.t_hr(baseline_idx) - opt.t_hr(opt_idx)) * 60;  % min
    
    % demonstrate delta_SOC
    pos = get(gca, 'Position');
    x_normalized = (0.1 * opt.t_hr(end) - min(xlim)) / diff(xlim) * pos(3) + pos(1);
    x = [x_normalized, x_normalized];
    
    y_low_normalized = (baseline_batt_min - min(ylim)) / diff(ylim) * pos(4) + pos(2);
    y_high_normalized = (opt_batt_min - min(ylim)) / diff(ylim) * pos(4) + pos(2);
    y = [y_low_normalized, y_high_normalized];
    
    annotation('doublearrow',x,y)
    
    text(0.1 * diff(xlim) + min(xlim), 0.5 * (opt_batt_min - baseline_batt_min) + baseline_batt_min,...
         ['$\Delta SOC=$', num2str(delta_soc), ' kwh'], 'Interpreter','latex', 'FontSize',14);
    
    % plot the difference of time reaching minimum battery
    x_start = opt.t_hr(opt_idx);
    x_end = x_start;
    y_start = min(ylim);
    y_end = max(ylim);
    line([x_start, x_end], [y_start, y_end],'Color','black','LineStyle','--');
    
    x_start = baseline.t_hr(baseline_idx);
    x_end = x_start;
    y_start = min(ylim);
    y_end = max(ylim);
    line([x_start, x_end], [y_start, y_end],'Color','black','LineStyle','--');
    
    % demonstrate delta_t
    x_low_normalized = (opt.t_hr(opt_idx) - min(xlim)) / diff(xlim) * pos(3) + pos(1);
    x_high_normalized = (opt.t_hr(baseline_idx) - min(xlim)) / diff(xlim) * pos(3) + pos(1);
    x = [x_low_normalized, x_high_normalized];
    y = [0.5, 0.5];
    
    annotation('doublearrow',x,y)
    
    text(0.5 * (baseline.t_hr(baseline_idx) - opt.t_hr(opt_idx)) + opt.t_hr(opt_idx),...
         0.5 * diff(ylim) + min(ylim), ['$\Delta t=$', num2str(delta_t), ' min'],...
         'Interpreter','latex', 'FontSize',14, 'HorizontalAlignment', 'center');
     
     
    % plot sun rise or sunset line
    x_sun = sun_time_before / (sun_time_before + sun_time_after) * diff(xlim) + min(xlim);
    line([x_sun, x_sun], [min(ylim), max(ylim)], 'Color','black','LineStyle','--');
    
    [~, sun_idx] = min(abs(opt.t_hr - x_sun));
    y_sun = opt.E_batt(sun_idx);
    
    x_sun_normalized = (x_sun - min(xlim)) / diff(xlim) * pos(3) + pos(1);
    y_sun_normalized = (y_sun - min(ylim)) / diff(ylim) * pos(4) + pos(2);
    
    sun_str = 'Sun Rise';
    
    temp = annotation('textarrow',[x_sun_normalized+0.02, x_sun_normalized],[y_sun_normalized+0.02, y_sun_normalized],...
                      'String',sun_str);
    temp.FontSize = 14;
    
    % plot(x_sun, y_sun, 'o');
    
    % concatenate title strings
    title_str = ', ';
    title_str = strcat(title_str, tst_option.wind_day, ', ');
    
    title_str = strcat(title_str, 'Wind magnitude=', num2str(ES.wind.Wmag), 'm/s, ');
    
    title_str = strcat(title_str, sun_str, ',[-', num2str(sun_time_before), ' min, ', num2str(sun_time_after), ' min]');
    
    title(['Performance metrics at ', tst_station.name, ', lat = ', num2str(tst_station.lat), ', lon = ', num2str(tst_station.lon), title_str], 'FontSize', 18);
    xlabel('t (h)');
    
    lgd = legend('Optimization Traj.', 'Baseline Traj');
    lgd.FontSize = 13;
    
end
