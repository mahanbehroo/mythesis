function euler_angles = rotation_matrix2euler_angles(rotation_matrix)

	%eul = [si theta phi];
	%eul = transpose(euler_angles);
	%rotation_matrix = eul2rotm(eul); % default is "ZYX" sequence

	%euler_angles = rotm2eul(rotation_matrix);

	% implementation for Octave: "ZYX" sequence

    theta = - asin(rotation_matrix(3,1)); 
    if theta > 0 
        si = - pi + atan(rotation_matrix(2,1) / rotation_matrix(1,1));
    else
    	si = atan(rotation_matrix(2,1) / rotation_matrix(1,1));
    end
	phi = atan(rotation_matrix(3,2) / rotation_matrix(3,3));

	euler_angles = [si theta phi];

	% delta = euler_angles_1 - euler_angles
	 

end

