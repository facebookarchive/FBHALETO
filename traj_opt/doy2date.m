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
function [dateV] = doy2date(doyV,yearV)
% Reference: https://www.mathworks.com/matlabcentral/fileexchange/24235-day-of-year-to-matlab-date?focused=5115824&tab=function

% DOY2DATE.m will convert a vector of day of year numbers and years
% and convert them to MATLAB date format.
%
% Sample Call:
%  doyV = [54;200.4315];
%  yearV = [2009;2009];
%  [dateV] = doy2date(doyV,yearV);
%
% Inputs:
%  doyV -> vector of day of year numbers (n x 1)
%  yearV -> vector of years (n x 1)
%
% Outputs:
%  dateV -> vector of MATLAB dates (n x 1)
%
% AUTHOR    : A. Booth (ashley [at] boothfamily [dot] com)
% DATE      : 22-May-2009 09:34:53
% Revision  : 1.00
% DEVELOPED : 7.4.0.287 (R2007a) Windows XP
% FILENAME  : doy2date.m

% Check size of incoming vectors
if (size(doyV, 1) == 1) | (size(doyV, 2) == 1) % Make sure only a nx1 vector
    if size(doyV, 1) < size(doyV, 2)
        doyV = doyV';
        colflip = 1; % take note if the rows were flipped to col
    else
        colflip = 0;
    end
else % check to see that vectors are columns:
    error('DOY vector can not be a matrix')
end

% year vector
if (size(yearV,1)== 1) | (size(yearV,2) == 1) % Make sure only a nx1 vector
    if size(yearV,1)<size(yearV,2)
        yearV = yearV';
%         colflip = 1; % take note if the rows were flipped to col
    else
%         colflip = 0;
    end
else % check to see that vectors are columns:
    error('Year vector can not be a matrix')
end

% Check to make sure sizes of the vectors are the same
if ~min(size(doyV) == size(yearV))
    error('Day of year vector and year vector must be the same size')
end


% Make year into date vector
z = zeros(length(yearV),5);
dv = horzcat(yearV,z);

% Calc matlab date
dateV = doyV + datenum(dv);

% flip output if input was flipped
if colflip
    dateV = dateV';
end


% disp('Completed doy2date.m')
% ===== EOF [doy2date.m] ======
