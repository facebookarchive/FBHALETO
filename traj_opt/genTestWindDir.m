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
function [ windDir ] = genTestWindDir( testStation, tst, alt )
% Use interpolation to generate the wind direction (radian)
% based on provided test statoin, test option, and altitude

    dayIdx = setTestDay(tst, testStation);

    wdir_level_idx = interp1(testStation.percs,[1:1:7], tst.wdir_level,'nearest');
    Wdir_alt = reshape(testStation.wnd_dir(dayIdx, wdir_level_idx, :),[1 6]) * pi / 180;
    dirAng = interp1(testStation.alts, Wdir_alt, alt, 'linear');

    % make sure the angle is between [-pi, pi] to be consistent with
    % Simulink model implementation
    windDir = atan2(sin(dirAng), cos(dirAng));

end
