function q = NewtonRaphson(q0, t)
% q = NewtonRaphson(q0, t)
% This function uses the Newton-Raphson method to solve a system of nonlinear equations.
%
% Inputs:
%   q0 - Initial approximation of the solution vector (absolute coordinates).
%   t  - The current time instant.
% Outputs:
%   q  - The solution vector that satisfies the nonlinear equations.

    % Initialize the solution vector with the initial approximation
    q = q0;

    % Calculate the initial constraint vector (residual) using the initial guess
    F = constraints(q, t);

    % Initialize the iteration counter
    counter = 1;

    % Iterative Newton-Raphson loop for solving nonlinear equations
    while (norm(F) > 1e-15) && (counter < 26)
        % Compute the constraint vector (residual) at the current estimate
        F = constraints(q, t);

        % Compute the Jacobian matrix for the current estimate
        Fq = Jacobian(q);

        % Update the solution vector using the Newton-Raphson method
        q = q - Fq \ F;

        % Increment the iteration counter
        counter = counter + 1;
    end

    % Check if the method failed to converge within the maximum allowed iterations
    if counter >= 25
        error('Warning: No convergence after 25 iterations at time = %.0f.', t);
    end
end


