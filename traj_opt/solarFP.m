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
function fp = solarFP(maj_axis, min_axis, orbit_dir, x_ctr, y_ctr, az)
%solarFP Summary of this function goes here
%   Detailed explanation goes here

nc = x_ctr;
ec = y_ctr;

rho = min_axis;
drho = maj_axis - min_axis;

nu_ctr = nc + drho * cos(az);
eu_ctr = ec + drho * sin(az);

nur = nu_ctr - rho * sin(az);
eur = eu_ctr + rho * cos(az);
nul = nu_ctr + rho * sin(az);
eul = eu_ctr - rho * cos(az);

nl_ctr = nc - drho * cos(az);
el_ctr = ec - drho * sin(az);

nlr = nl_ctr - rho * sin(az);
elr = el_ctr + rho * cos(az);
nll = nl_ctr + rho * sin(az);
ell = el_ctr - rho * cos(az);

% orbit direction: 1 - CCW, 0 - CW
if (orbit_dir)  % CCW
    fp = [1	6	2	nur eur 0	0	0	0	1	0;
        2	1	3   nu_ctr eu_ctr 1	min_axis	1	180	3	5;
        3	2	4   nul eul 0	0	0	0	1	0;
        4	3	5   nll ell 0	0	0	0	1	0;
        5	4	6   nl_ctr el_ctr 1	min_axis	1	180	3	5;
        6	5	1   nlr elr 0	0	0	0	1	0];
else    % CW
    fp = [1	6	2	nul eul 0	0	0	0	1	0;
        2	1	3	nu_ctr eu_ctr 1	min_axis	0	180	3	5;
        3	2	4	nur eur 0	0	0	0	1	0;
        4	3	5	nlr elr 0	0	0	0	1	0;
        5	4	6	nl_ctr el_ctr 1	min_axis	0	180	3	5;
        6	5	1	nll ell 0	0	0	0	1	0];
end


end
