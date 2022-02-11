function rotation_matrix = euler_angles2rotation_matrix(euler_angles)

	%eul = [si theta phi];
	% eul = transpose(euler_angles);
	% rotation_matrix = eul2rotm(eul); % default is "ZYX" sequence

	% implementation for Octave version: "ZYX" or "3-2-1" sequence

	rotation_matrix = zeros(3,3);

	si = euler_angles(1);
	theta = euler_angles(2);
	phi = euler_angles(3);
	rotation_matrix(1,1) = cos(theta) * cos(si);
	rotation_matrix(1,2) = - cos(phi) * sin(si) + sin(phi) * sin(theta) * cos(si);
	rotation_matrix(1,3) = sin(phi) * sin(si) + cos(phi) * sin(theta) * cos(si);
	rotation_matrix(2,1) = cos(theta) * sin(si);
	rotation_matrix(2,2) = cos(phi) * cos(si) + sin(phi) * sin(theta) * sin(si);
	rotation_matrix(2,3) = - sin(phi) * cos(si) + cos(phi) * sin(theta) * sin(si);
	rotation_matrix(3,1) = - sin(theta);
	rotation_matrix(3,2) = sin(phi) * cos(theta);
	rotation_matrix(3,3) = cos(phi) * cos(theta); 

	% delta = rotation_matrix_1 - rotation_matrix
end