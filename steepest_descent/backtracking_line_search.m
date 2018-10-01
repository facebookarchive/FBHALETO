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
function  [step_size, iter, flag]  = backtracking_line_search( fun, x0, gradient, alpha, beta )
% backtracking to to determine the step size to take for the search direction

    t = 1;
    step_size = 1;

    iter = 0;
    max_iter = 25;

    flag = 0;

    fun_val0 = fun(x0);
    norm_0 = norm(gradient);

    while iter < max_iter
        fun_val = fun(x0 - t * gradient);
        if fun_val0 - fun_val >= alpha * t * norm_0 * norm_0
            % if the current step size t can decrease objective function
            % enough, return this step size
            step_size = t;
            return;
        end
        
        t = t * beta;
        iter = iter + 1;
    end

    % if reach here, max_iter is reached,
    % set flag to be 1, meaning no sufficient descent
    flag = 1;

end
