function rotation_matrix = euler_angles2rotation_matrix(euler_angles)

	%eul = [si theta phi];
	eul = transpose(euler_angles);
	rotation_matrix = eul2rotm(eul); % default is "ZYX" sequence

end