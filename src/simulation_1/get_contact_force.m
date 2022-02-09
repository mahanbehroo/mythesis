function contact_force = get_contact_force(object1, object2, contact_situation, z)
    % NEED z to calculate u_normal vector

    index = 21;
	r_db = [z(19:21)];
    v_db = [z(16:18)];
	r_sc = [z(index + 19:index + 21)];	
	v_sc = [z(index + 16:index + 18)];
    r_s2d = r_sc - r_db;
    u_normal = r_s2d /norm(r_s2d);
	
	
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
    nn = 1.5;
    alpha = 0.15;
    %--------------
	r= (R_1^-1+R_2^-1)^-1;
	h_1= (1-nou_1^2)/(pi*E_1);
	h_2= (1-nou_2^2)/(pi*E_2);
	k_c= (4/(3*pi))*(sqrt(r)/(h_1+h_2));
	lambda = 1.5*alpha*k_c;

    if delta > 0
	    normal_force = k_c*delta^nn+lambda*delta^nn*delta_dot
        contact_force = normal_force * u_normal;
    else
        normal_force = 0;
        contact_force = normal_force * u_normal;
    end

    if object2 == 'spacecraft'		

		contact_force =  contact_force;

	else
		contact_force = - contact_force;

    end

end