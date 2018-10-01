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

% This is a test script of steepest descent method
% Directly run it to solve a simple convex optimization problem


clear;

lb = [-Inf, -Inf, -Inf];
ub = [Inf, Inf, Inf];

x0 = [-32, 40, 3];

% simple convex function for test
% optimal solution is [100, 233, -Inf]
fun = @(x) (x(1) - 100)^2 + (x(2)-233)^2 + exp((x(3) - 2));


% for additional custom options, refer to steepest_descent.m
options = {'use_parallel', false};
[ x, fval, eflag ] = steepest_descent( fun, x0, lb, ub,options )
