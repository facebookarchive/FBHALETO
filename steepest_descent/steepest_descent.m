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
function [ x, fval, eflag ] = steepest_descent( fun, x0, lb, ub, options)
% This is an implementation of steepest descent method with projection

    % user can specify four options for now, add more options in the map
    keySet = {'max_iter', 'grad_tol', 'grad_delta', 'step_size_tol', 'use_parallel'};
    valueSet = {10000, 1.0e-6, 1.0e-8, 1.0e-10, false};
    alg_opt = containers.Map(keySet,valueSet);
    eflag = 5; % 5 means terminate abnormally, need to investigate into solver.

    if mod(length(options), 2) ~= 0
        disp('Error: Customized options have to be paired.');
    else
        for i = 1 : 2 : length(options)
            key = options{i};
            val = options{i + 1};
            alg_opt(key) = val;
        end
    end

    % make sure x0 is feasible
    if ~isInRange(x0, lb, ub)
        x0 = projection(x0, lb, ub);
    end

    % spcify option values for the solver
    max_iter      = alg_opt('max_iter');
    grad_tol      = alg_opt('grad_tol');
    grad_delta    = alg_opt('grad_delta');
    step_size_tol = alg_opt('step_size_tol');
    use_parallel  = alg_opt('use_parallel');


    % preparation for first iteration
    k = 1;
    grad = cal_gradient(fun, x0, grad_delta, use_parallel);
    norm_grad = norm(grad);
    x_prev = x0;

    disp(['Iter=0', '  norm_gradient=', num2str(norm_grad),...
          '  fval=', num2str(fun(x0))]);

    % algorithm iterations
    while (k <= max_iter && norm_grad >= grad_tol)
        
        % Accelerated gradient method
        % momentum term comes from slide 5 of https://fburl.com/nkbso0lo
        y = x0 + (k - 2) / (k + 1) * (x0 - x_prev);
        
        grad = cal_gradient(fun, y, grad_delta, use_parallel);
        % user can adjust alpha/beta which are both set to be 0.5 here
        [step_size, line_search_iters, line_search_flag] = backtracking_line_search(fun, y, grad, 0.5, 0.5);
        
        if line_search_flag == 1
            disp('Algorithm terminated because it can not find sufficient descent on the search direction.');
            eflag = 2;
            x = x0;
            fval = fun(x);
            break;
        end


        x0_prime = y - step_size * grad;
        x0_prime = projection(x0_prime, lb, ub);

        norm_grad = norm(grad);
        
        disp(['Iter=', num2str(k), '  norm_gradient=', num2str(norm_grad),...
              '  step_size=', num2str(step_size),...
              '  line search iterations = ', num2str(line_search_iters),...
              '  fval=', num2str(fun(x0))]);
        
        x_prev = x0;
        x0 = x0_prime;
        k = k + 1;
    end

    if k >= max_iter
        disp('Algorithm terminated because it reaches the max iteration number');
        eflag = 1;
    end
    if norm(grad) < grad_tol
        disp('Algorithm terminated because norm of gradient is small, local optimum reached.')
        eflag = 0;
    end

    x = x0;
    fval = fun(x);

end
