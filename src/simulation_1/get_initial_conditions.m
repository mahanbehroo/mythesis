function initial_conditions = get_initial_conditions()

	initial_orientation = [0;0;0];
	initial_euler_angles = [1;2;3]; % [si, theta, phi]
	initial_rates_B = [1;1;0.5]; % rad/s - w_body represented in body frame B

	%initial_rotation_matrix = [1 0 0; 0 1 0; 0 0 1]; % identity for test0, care for reshape usage!!!!!
	initial_rotation_matrix = euler_angles2rotation_matrix(initial_euler_angles);
	%initial_rates_R = initial_rotation_matrix * initial_rates_B;

	initial_position = [0;0;0];
	initial_velocity = [1;0;1];

	initial_conditions_db = [initial_rates_B; initial_orientation; reshape(transpose(initial_rotation_matrix),[9,1]); initial_velocity; initial_position];
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	initial_orientation_sc = [0;0;0];
	initial_euler_angles_sc = [0;0;0]; % [si, theta, phi]
	initial_rates_Sc2R = [1;1;0.5]; % rad/s - w_body represented in body frame B

	%initial_rotation_matrix = [1 0 0; 0 1 0; 0 0 1]; % identity for test0, care for reshape usage!!!!!
	initial_rotation_matrix_Sc2R = euler_angles2rotation_matrix(initial_euler_angles_sc);
	%initial_rates_R = initial_rotation_matrix * initial_rates_B;

	initial_position_Sc = [1;1;1];
	initial_velocity_Sc = [0;-1;0];

	initial_conditions_sc = [initial_rates_Sc2R; initial_orientation_sc; reshape(transpose(initial_rotation_matrix_Sc2R),[9,1]); initial_velocity_Sc; initial_position_Sc];
	initial_conditions = [initial_conditions_db;initial_conditions_sc];

end