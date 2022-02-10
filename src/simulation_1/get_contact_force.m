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
    u_tangent = t_s2d; 

    

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