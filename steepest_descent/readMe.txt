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

This is an implementation of steepest descent with projection, specifically for
solving convex optimization with boxed constraints.

Refer to https://www.cs.cmu.edu/~ggordon/10725-F12/slides/09-acceleration.pdf for the main body of the algorithm, especially slides 5.


How to use the algorithm?

1. Run test_steepest_descent.m to make sure solver works.

2. To invoke solver, the entry point is steepest_descent.m.
