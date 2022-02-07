function [theta, e_hat] = rotation_matrix2euler_angleAxis(rotation_matrix)
	% in this function we implement the transform from a rotation matrix to 
	% a single axis (Euller Axis) and an angle of rotation to go from frame A to fram B
	% Eigendecomposition of the rotation matrix yields the eigenvalues 1 and cos θ ± i sin θ. 
	% The Euler axis is the eigenvector corresponding to the eigenvalue of 1, 
	% and θ can be computed from the remaining eigenvalues.
	% The Euler axis can be also found using singular value decomposition
	% since it is the normalized vector spanning the null-space
	% of the matrix I − A.

	%  R_A2B ---> e_hat = [e1;e2;e3] and theta

	theta = acos((1/2)*(trace(rotation_matrix)-1));    % amount of rotation in radians 
	e1 = (rotation_matrix(3,2) - rotation_matrix(2,3))/(2 * sin(theta)); % 1st element of Euler Axis
	e2 = (rotation_matrix(1,3) - rotation_matrix(3,1))/(2 * sin(theta));	% 2nd element of Euler Axis
	e3 = (rotation_matrix(2,1) - rotation_matrix(1,2))/(2 * sin(theta));	% 3rd element of Euler Axis
	e_hat = [e1;e2;e3];

end