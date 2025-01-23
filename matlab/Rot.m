function R = Rot(fi)
% R = Rot(fi)
% This function computes a 2x2 rotation matrix for a given angle.
%
% Input:
%   fi - The angle of rotation in radians.
%
% Output:
%   R  - The 2x2 rotation matrix.
%
% The rotation matrix is used to transform vectors in 2D space by rotating them
% counterclockwise by the angle `fi`.

    % Construct the 2x2 rotation matrix
    % The matrix rotates vectors counterclockwise in the xy-plane
    R = [cos(fi), -sin(fi);  % First row: [cos(fi), -sin(fi)]
         sin(fi),  cos(fi)]; % Second row: [sin(fi), cos(fi)]
end
