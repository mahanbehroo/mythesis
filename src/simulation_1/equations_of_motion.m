function dz = equations_of_motion(t,z)

	

	moment_of_inerta = get_moment_of_inertia('debris0000');

	I_11 = moment_of_inerta(1,1);
	I_22 = moment_of_inerta(2,2);
	I_33 = moment_of_inerta(3,3);

	omega_1 = z(1); % omega_Body_wtr_Reference represented in Body frame
	omega_2 = z(2);
	omega_3 = z(3);

	
    % -----------------------------------------------------
    % -----------------------------------------------------------
    object_properties = get_object_properties('debris0000');

    mass = object_properties(1);

    contact_situation = detect_contact(t, z);
    if contact_situation(1) == 1
        [normal_force_imposedOn_debris, friction_imposedOn_debris] = get_contact_force('spacecraft', 'debris0000', contact_situation, z);
    else
        friction_imposedOn_debris = [0;0;0];
        normal_force_imposedOn_debris = [0;0;0];
    end
    % -----------------------------------------------------

    % implement friction here!
    %friction_imposedOn_debris = [0;0;0];



    net_force = normal_force_imposedOn_debris + friction_imposedOn_debris;
    % -----------------------------------------------------------
    a = object_properties(2);
    if norm(friction_imposedOn_debris) > 0
        lever_db = a * (normal_force_imposedOn_debris/norm(normal_force_imposedOn_debris));
    else
        lever_db =[0; 0; 0];
    end
    moment_in_R = get_total_moment(t, 'debris0000', friction_imposedOn_debris, lever_db);
    
    rotation_matrix = zeros(3,3);

    rotation_matrix(1,1) = z(7,1);
    rotation_matrix(1,2) = z(8,1);
    rotation_matrix(1,3) = z(9,1);
    rotation_matrix(2,1) = z(10,1);
    rotation_matrix(2,2) = z(11,1);
    rotation_matrix(2,3) = z(12,1);
    rotation_matrix(3,1) = z(13,1);
    rotation_matrix(3,2) = z(14,1);
    rotation_matrix(3,3) = z(15,1);

    if det(rotation_matrix)== 1
        test_determinant = 0;
    else
        test_determinant = det(rotation_matrix);
    end

    moment_in_B = rotation_matrix * moment_in_R;
	M_1 = moment_in_B(1);
	M_2 = moment_in_B(2);
	M_3 = moment_in_B(3);


	dz_rot_obj = zeros(6,1); % instantiate a zero vector for elements

	% I_11 * omega_dot_1 = M_1 + (I_22 - I_33) * omega_2 * omega_3
	dz_rot_obj(1,1) = (1/I_11) * (M_1 + (I_22 - I_33) * omega_2 * omega_3);
	% I_22 * omega_dot_2 = M_2 + (I_33 - I_11) * omega_3 * omega_1 
	dz_rot_obj(2,1) = (1/I_22) * (M_2 + (I_33 - I_11) * omega_3 * omega_1);
	% I_33 * omega_dot_3 = M_3 + (I_11 - I_22) * omega_1 * omega_2
	dz_rot_obj(3,1) = (1/I_33) * (M_3 + (I_11 - I_22) * omega_1 * omega_2);

	dz_rot_obj(4,1) = z(1,1);
	dz_rot_obj(5,1) = z(2,1);
	dz_rot_obj(6,1) = z(3,1);


	


	%%%%%%%%%%%%%%%%%%%%

	dz(1,1) = dz_rot_obj(1,1);
	dz(2,1) = dz_rot_obj(2,1);
	dz(3,1) = dz_rot_obj(3,1);
	dz(4,1) = dz_rot_obj(4,1);
	dz(5,1) = dz_rot_obj(5,1);
	dz(6,1) = dz_rot_obj(6,1);



	%%%%%%%%%%%%%%%%%%%%%%
	%    d/dt (rotation matrix) = omega skew symmetric * rotation matrix

	% it's not most efficient but it behaves well. better to use quaternians instead.

	%--------------------


	%rotation_matrix_dot = zeros(3,3);

	%rotation_matrix_dot(1,1) = - omega_3 * rotation_matrix(2,1) + omega_2 * rotation_matrix(3,1);
	%rotation_matrix_dot(1,2) = - omega_3 * rotation_matrix(2,2) + omega_2 * rotation_matrix(3,2); 
	%rotation_matrix_dot(1,3) = - omega_3 * rotation_matrix(2,3) + omega_2 * rotation_matrix(3,3); 

	% rotation_matrix_dot(2,1) = omega_3 * rotation_matrix(1,1) - omega_1 * rotation_matrix(3,1);
	% rotation_matrix_dot(2,2) = omega_3 * rotation_matrix(1,2) - omega_1 * rotation_matrix(3,2);
	% rotation_matrix_dot(2,3) = omega_3 * rotation_matrix(1,3) - omega_1 * rotation_matrix(3,3);

	% rotation_matrix_dot(3,1) = - omega_2 * rotation_matrix(1,1) + omega_1 * rotation_matrix(2,1); 
	% rotation_matrix_dot(3,2) = - omega_2 * rotation_matrix(1,2) + omega_1 * rotation_matrix(2,2); 
	% rotation_matrix_dot(3,3) = - omega_2 * rotation_matrix(1,3) + omega_1 * rotation_matrix(2,3); 

    rotation_matrix_dot =zeros(3,3);
    rotation_matrix_dot = [0 -omega_3 omega_2; omega_3 0 -omega_1; -omega_2 omega_1 0] * rotation_matrix;

    % test_matrix = rotation_matrix_dot_new - rotation_matrix_dot;
    % if test_matrix == zeros(3,3)
    %     hhh= 0;
    % else
    %     hhh = test_matrix
    % end




	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    dz(7,1) = rotation_matrix_dot(1,1);
    dz(8,1) = rotation_matrix_dot(1,2);
    dz(9,1) = rotation_matrix_dot(1,3);
    dz(10,1) = rotation_matrix_dot(2,1); 
    dz(11,1) = rotation_matrix_dot(2,2);
    dz(12,1) = rotation_matrix_dot(2,3);
    dz(13,1) = rotation_matrix_dot(3,1);
    dz(14,1) = rotation_matrix_dot(3,2);
    dz(15,1) = rotation_matrix_dot(3,3);

	%%%%%%%%%%%%%%%%%%%%%%




	dz(16,1) = (1/mass) * net_force(1);
    dz(17,1) = (1/mass) * net_force(2);
    dz(18,1) = (1/mass) * net_force(3);
    dz(19,1) = z(16,1);
	dz(20,1) = z(17,1);
	dz(21,1) = z(18,1);

	%--------------------------------------------------------------%
	%------------- End of Debris related states -------------------%
	%--------------------------------------------------------------%

	% Start priducing spacecraft related states
	% index is used to reset for used slots in dz
	index = 21; % 6+9+6 rotation + rotation matrix + translation

	moment_of_inerta_sc = get_moment_of_inertia('spacecraft');

	I_11 = moment_of_inerta_sc(1,1);
	I_22 = moment_of_inerta_sc(2,2);
	I_33 = moment_of_inerta_sc(3,3);

	omega_1 = z(index + 1,1);
	omega_2 = z(index + 2,1);
	omega_3 = z(index + 3,1);

    % -----------------------------------------------------
    % -----------------------------------------------------------

    	%%%%%%%%%%%%%%%%%%%%%%
	object_properties = get_object_properties('spacecraft');
    mass = object_properties(1);
	normal_force_imposedOn_spacecraft = - normal_force_imposedOn_debris;
    friction_imposedOn_spacecraft = - friction_imposedOn_debris;
    % -------
    net_force = normal_force_imposedOn_spacecraft + friction_imposedOn_spacecraft;
    %--------
    a = object_properties(2);

    if norm(friction_imposedOn_spacecraft) > 0
        lever_db = a * (normal_force_imposedOn_spacecraft/norm(normal_force_imposedOn_spacecraft));
    else
        lever_db =[0; 0; 0];
    end
    %lever_db = a * (normal_force_imposedOn_spacecraft/norm(normal_force_imposedOn_spacecraft)); 
    moment_in_R = get_total_moment(t, 'spacecraft', friction_imposedOn_spacecraft, lever_db);

    rotation_matrix = zeros(3,3);

    rotation_matrix(1,1) = z(index + 7,1);
    rotation_matrix(1,2) = z(index + 8,1);
    rotation_matrix(1,3) = z(index + 9,1);
    rotation_matrix(2,1) = z(index + 10,1);
    rotation_matrix(2,2) = z(index + 11,1);
    rotation_matrix(2,3) = z(index + 12,1);
    rotation_matrix(3,1) = z(index + 13,1);
    rotation_matrix(3,2) = z(index + 14,1);
    rotation_matrix(3,3) = z(index + 15,1);

    if det(rotation_matrix)== 1
        test_determinant = 0;
    else
        test_determinant = det(rotation_matrix);
    end



    moment_in_B = rotation_matrix * moment_in_R;
	M_1 = moment_in_B(1);
	M_2 = moment_in_B(2);
	M_3 = moment_in_B(3);

	dz_rot_sc = zeros(6,1); % instantiate a zero vector for elements

	% I_11 * omega_dot_1 = M_1 + (I_22 - I_33) * omega_2 * omega_3
	dz_rot_sc(1,1) = (1/I_11) * (M_1 + (I_22 - I_33) * omega_2 * omega_3);
	% I_22 * omega_dot_2 = M_2 + (I_33 - I_11) * omega_3 * omega_1 
	dz_rot_sc(2,1) = (1/I_22) * (M_2 + (I_33 - I_11) * omega_3 * omega_1);
	% I_33 * omega_dot_3 = M_3 + (I_11 - I_22) * omega_1 * omega_2
	dz_rot_sc(3,1) = (1/I_33) * (M_3 + (I_11 - I_22) * omega_1 * omega_2);

	dz_rot_sc(4,1) = z(index + 1,1);
	dz_rot_sc(5,1) = z(index + 2,1);
	dz_rot_sc(6,1) = z(index + 3,1);


	


	%%%%%%%%%%%%%%%%%%%%

	dz(index + 1,1) = dz_rot_sc(1,1);
	dz(index + 2,1) = dz_rot_sc(2,1);
	dz(index + 3,1) = dz_rot_sc(3,1);
	dz(index + 4,1) = dz_rot_sc(4,1);
	dz(index + 5,1) = dz_rot_sc(5,1);
	dz(index + 6,1) = dz_rot_sc(6,1);


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





	%rotation_matrix_dot = zeros(3,3);

	%rotation_matrix_dot(1,1) = - omega_3 * rotation_matrix(2,1) + omega_2 * rotation_matrix(3,1);
	%rotation_matrix_dot(1,2) = - omega_3 * rotation_matrix(2,2) + omega_2 * rotation_matrix(3,2); 
	%rotation_matrix_dot(1,3) = - omega_3 * rotation_matrix(2,3) + omega_2 * rotation_matrix(3,3); 

	% rotation_matrix_dot(2,1) = omega_3 * rotation_matrix(1,1) - omega_1 * rotation_matrix(3,1);
	% rotation_matrix_dot(2,2) = omega_3 * rotation_matrix(1,2) - omega_1 * rotation_matrix(3,2);
	% rotation_matrix_dot(2,3) = omega_3 * rotation_matrix(1,3) - omega_1 * rotation_matrix(3,3);

	% rotation_matrix_dot(3,1) = - omega_2 * rotation_matrix(1,1) + omega_1 * rotation_matrix(2,1); 
	% rotation_matrix_dot(3,2) = - omega_2 * rotation_matrix(1,2) + omega_1 * rotation_matrix(2,2); 
	% rotation_matrix_dot(3,3) = - omega_2 * rotation_matrix(1,3) + omega_1 * rotation_matrix(2,3); 

    rotation_matrix_dot =zeros(3,3);
    rotation_matrix_dot = [0 -omega_3 omega_2; omega_3 0 -omega_1; -omega_2 omega_1 0] * rotation_matrix;

    % test_matrix = rotation_matrix_dot_new - rotation_matrix_dot;
    % if test_matrix == zeros(3,3)
    %     hhh= 0;
    % else
    %     hhh = test_matrix
    % end




	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    dz(index + 7,1) = rotation_matrix_dot(1,1);
    dz(index + 8,1) = rotation_matrix_dot(1,2);
    dz(index + 9,1) = rotation_matrix_dot(1,3);
    dz(index + 10,1) = rotation_matrix_dot(2,1); 
    dz(index + 11,1) = rotation_matrix_dot(2,2);
    dz(index + 12,1) = rotation_matrix_dot(2,3);
    dz(index + 13,1) = rotation_matrix_dot(3,1);
    dz(index + 14,1) = rotation_matrix_dot(3,2);
    dz(index + 15,1) = rotation_matrix_dot(3,3);



	dz(index + 16,1) = (1/mass) * net_force(1);
    dz(index + 17,1) = (1/mass) * net_force(2);
    dz(index + 18,1) = (1/mass) * net_force(3);
    dz(index + 19,1) = z(index + 16,1);
	dz(index + 20,1) = z(index + 17,1);
	dz(index + 21,1) = z(index + 18,1);

end