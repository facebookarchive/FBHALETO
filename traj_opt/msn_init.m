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
function [ICs, fp_courses, fp_coords] = msn_init(fp, init_wp, x_0, y_0)
%MSN_INIT Computes mission initial conditions given a flight plan
%

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

ICs = [to_init next_init d_init dchi0];

end
