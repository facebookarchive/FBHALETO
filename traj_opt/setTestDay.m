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
function dayIdx = setTestDay(tst, testStation)
% Generate the day index given the test options and test station data

    if (tst.location(3) == 'N')
        % Northern hemisphere
        switch tst.wind_day
            case 'Winter'
                dayIdx = testStation.dayIdx(4);
            case 'Fall'
                dayIdx = testStation.dayIdx(3);
            case 'Summer'
                dayIdx = testStation.dayIdx(2);
            case 'Spring'
                dayIdx = testStation.dayIdx(1);
            otherwise
                dayIdx = tst.specDay;
        end
    else
        % Southern hemisphere
        switch tst.wind_day
            case 'Winter'
                dayIdx = testStation.dayIdx(2);
            case 'Fall'
                dayIdx = testStation.dayIdx(1);
            case 'Summer'
                dayIdx = testStation.dayIdx(4);
            case 'Spring'
                dayIdx = testStation.dayIdx(3);
            otherwise
                dayIdx = tst.specDay;
        end
    end
end
