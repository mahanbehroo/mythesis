function euler_angles = rotation_matrix2euler_angles(rotation_matrix)

	%eul = [si theta phi];
	%eul = transpose(euler_angles);
	%rotation_matrix = eul2rotm(eul); % default is "ZYX" sequence

	euler_angles = rotm2eul(rotation_matrix);

end

