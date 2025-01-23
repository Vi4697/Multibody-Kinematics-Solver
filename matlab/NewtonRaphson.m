function q = NewtonRaphson(q0, t, P)
% NewtonRaphson(q0, t)
% Solves the constraint equations using the Newton-Raphson method.
%
% Inputs:
%   q0 - Initial guess for the state variables (absolute coordinates).
%   t  - Current time (for time-dependent constraints).
%
% Output:
%   q  - Solution for the state variables after convergence.

    maxIterations = 30;  % Maximum number of iterations allowed
    tolerance = 1e-8;    % Convergence tolerance for constraints
    q = q0;              % Initialize q with the initial guess
    
    for iter = 1:maxIterations
        % Calculate the constraints and Jacobian
        F = constraints(q, t, P);
        Fq = Jacobian(q);
        
        % Debug: Log iteration info
        %fprintf('Iteration: %d, Norm of F: %e\n', iter, norm(F));
        
        % Check for convergence
        if norm(F) < tolerance
            fprintf('Converged after %d iterations.\n', iter);
            return;
        end
        
        % Update q using Newton-Raphson step
        try
            delta_q = -Fq \ F; % Solve the linear system for the update step
            q = q + delta_q;
        catch ME
            % Handle singular Jacobian or other issues
            warning('Newton-Raphson failed at iteration %d due to Jacobian issues.', iter);
            disp('Jacobian Matrix:');
            disp(Fq);
            disp('Constraint Vector:');
            disp(F);
            rethrow(ME);
        end
    end

    % If maxIterations is reached without convergence
    warning('No convergence after %d iterations at time = %f.', maxIterations, t);
    disp('Final Norm of F:');
    disp(norm(F));
    disp('Final Constraint Vector F:');
    disp(F);
    disp('Final State Vector q:');
    disp(q);
end
