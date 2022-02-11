function [normal_force, friction] = get_contact_force(object1, object2, contact_situation, z)
    % NEED z to calculate u_normal vector
    %normal_force =  [0;0;0];
    %friction = [0;0;0];

    index = 21;
	r_db = [z(19:21)];
    v_db = [z(16:18)];
	r_sc = [z(index + 19:index + 21)];	
	v_sc = [z(index + 16:index + 18)];
    r_s2d = r_sc - r_db;
    u_normal = r_s2d /norm(r_s2d);

    v_s2d = v_sc - v_db;
    u_v_s2d = v_s2d / norm(v_s2d);

    q_s2d = cross(u_normal, u_v_s2d);
    t_s2d = - cross(q_s2d, u_normal);

    omega_db = [z(1:3)];

    debri_properties = get_object_properties('debris0000');
    a = debri_properties(2);
    r_contact_point_db = a * u_normal;

    v_contact_point_db = cross(omega_db, r_contact_point_db);
    
    spacecraft_properties = get_object_properties('spacecraft');
    a2 = spacecraft_properties(2);
    r_contact_point_sc = - a2 * u_normal;
    
    omega_sc = [z(index + 1:index + 3)];
    v_contact_point_sc = cross(omega_sc, r_contact_point_sc);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

    matrix_1 = rotation_matrix;


    rotation_matrix_2 = zeros(3,3);

    rotation_matrix_2(1,1) = z(index + 7,1);
    rotation_matrix_2(1,2) = z(index + 8,1);
    rotation_matrix_2(1,3) = z(index + 9,1);
    rotation_matrix_2(2,1) = z(index + 10,1);
    rotation_matrix_2(2,2) = z(index + 11,1);
    rotation_matrix_2(2,3) = z(index + 12,1);
    rotation_matrix_2(3,1) = z(index + 13,1);
    rotation_matrix_2(3,2) = z(index + 14,1);
    rotation_matrix_2(3,3) = z(index + 15,1);

    matrix_2 = rotation_matrix_2;
    
    v_contact_point_db = matrix_1' * v_contact_point_db;

    v_contact_point_sc = matrix_2' * v_contact_point_sc;

    %%%%%%%%%%%%%%%%%%%%%%%%%%5%%

    relative_rotation_component_vector = v_contact_point_sc - v_contact_point_db;
    dummy = relative_rotation_component_vector;
    
    if norm(dummy) > 0
        relative_rotation_component_unit_vector = dummy / norm(dummy);
    else
        relative_rotation_component_unit_vector = [0;0;0];
    end

     
    u_tangent = t_s2d + relative_rotation_component_unit_vector ; 
    u_tangent = u_tangent / norm(u_tangent);

    

    %relative_tang_motion_rate = dot(v_s2d, r_s2d) * (r_s2d/norm(r_s2d)) + cross(omega, r_contact_point);
    isSlipping = 0; %%%

    %u_tangent = [0;0;0];
	
	
    delta = contact_situation(2);
    delta_dot = contact_situation(3);

	
    object1_properties = get_object_properties(object1);
    object2_properties = get_object_properties(object2);
    R_1 = object1_properties(2);
    R_2 = object2_properties(2);
    nou_1 = 0.35;
    nou_2 = 0.48; % poisson ratio of silicone rubber
    E_1 = 7e+10; % pascal young modulus of aluminium alloy
    E_2 = 1e+6; % young modulus of silicone rubber


    % Continuous comliant model for normal force.
    nn = 1.5;
    alpha = 0.15;
    %--------------
	r= (R_1^-1+R_2^-1)^-1;
	h_1= (1-nou_1^2)/(pi*E_1);
	h_2= (1-nou_2^2)/(pi*E_2);
	k_c= (4/(3*pi))*(sqrt(r)/(h_1+h_2));
	lambda = 1.5*alpha*k_c;

    if delta > 0
	    normal_force_mag = k_c*delta^nn+lambda*delta^nn*delta_dot;
        normal_force = normal_force_mag * u_normal;
        if isSlipping == true
            miu_k = 1.1; % dummy nonzero value
            friction_mag = miu_k * normal_force_mag;
            friction = friction_mag * u_tangent;
        else
            miu_s = 1.2; % dummy nonzero value
            friction_mag = miu_s * normal_force_mag;
            friction = friction_mag * u_tangent;
        end

    else
        normal_force = 0;
        normal_force = normal_force * u_normal;
        friction_mag = 0;
        friction = friction_mag * u_tangent;
    end
   

    if object2 == 'spacecraft'		

		normal_force =  normal_force;
        friction = friction;

	else
		normal_force = - normal_force;
        friction = - friction;

    end

end