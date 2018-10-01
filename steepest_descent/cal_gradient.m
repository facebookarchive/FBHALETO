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
function gradient = cal_gradient( fun, x, delta, use_parallel )
% This function calculates the gradient of the fun at point x, using
% forward method.

	fun_val0 = fun(x);              % function value at point x
	n = length(x);                  % number of variables
	gradient = zeros(size(x));      % initilize the gradient vector

	if use_parallel
		parfor i = 1 : n
		    temp_x = x;
		    temp_x(i) = temp_x(i) + delta;
		    temp_fun = fun;
		    gradient(i) = (temp_fun(temp_x) - fun_val0) / delta;
		end
	else
		for i = 1 : n
		    x(i) = x(i) + delta;
			gradient(i) = (fun(x) - fun_val0) / delta;
			x(i) = x(i) - delta;
		end
	end

end
