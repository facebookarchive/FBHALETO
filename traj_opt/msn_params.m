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
function [fp_courses,fp_coords] = msn_params(fp)
%MSN_PARAMS Precompute mission courses
%
[num_wp,~] = size(fp);

chi_nom    = zeros(num_wp,1);	% course of this leg (to, from)
chi_adj    = zeros(num_wp,1);   % course of next leg (next, to)
nxt_chi    = zeros(num_wp,1);   % course of next leg (next, to)
dchi       = zeros(num_wp,1);   % change in course

x_entry = zeros(num_wp,1);
y_entry = zeros(num_wp,1);

for i = 1:num_wp,
    
    % Compute chi, next_chi, delta_chi
    to_wp = fp(i, 1);
    from_wp = fp(i, 2);
    
    to_x = fp(to_wp, 4);
    to_y = fp(to_wp, 5);
    from_x = fp(from_wp, 4);
    from_y = fp(from_wp, 5);
    
    dy = to_y - from_y;
    dx = to_x - from_x;
    chi_nom(i) = atan2(dy,dx);
    
    % Compute entry point for intercept patternholds
    % whenever intercept and orbit are both defined
    if (fp(i,10) == 3)
        % Circle parameters
        rho = fp(to_wp, 7);
        D = sqrt((from_x - to_x)^2 + (from_y - to_y)^2);
        % problem when rho > D
        if (rho > D)
            gamma = pi/2;
            De = 0.0;
        else
            gamma = pi/2 - acos(rho / D);
            De = sqrt(D*D - rho*rho);
        end
        
        if (fp(to_wp,8) > 0)    % CCW
            chi_adj(i) = chi_nom(i) + gamma;
        else                    % CW
            chi_adj(i) = chi_nom(i) - gamma;
        end
        
        y_entry(i) = from_y + De * sin(chi_adj(i));
        x_entry(i) = from_x + De * cos(chi_adj(i));
    else
        chi_adj(i) = chi_nom(i);
        x_entry(i) = to_x;
        y_entry(i) = to_y;
    end
end

for i = 1:num_wp,
    % Compute next_chi, delta_chi
	next_wp = fp(i, 3);
    nxt_chi(i) = chi_adj(next_wp);
end

% Compute dchi using the actual entry points
%nxt_chi = [chi_adj(2:end);chi_adj(1)];

delta_crs = nxt_chi - chi_adj;
dchi = atan2(sin(delta_crs),cos(delta_crs));

fp_courses = [chi_adj nxt_chi dchi];
fp_coords = [x_entry y_entry];
end
