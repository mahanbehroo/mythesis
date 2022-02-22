function  contact_situation = detect_contact(t, z)

	object_1 = get_object_properties('spacecraft');
	object_2 = get_object_properties('debris0000');

	object1_isMembrane = 1; % temporary
	


	if object_1(2) == object_1(3) && object_1(2) == object_1(4)
		object1_isSphere = true;
	else
		object1_isEllips = true;
        object1_isSphere = false;
	end

	if object_2(2) == object_2(3) && object_2(2) == object_2(4)
		object2_isSphere = true;
	else
		object2_isEllips = true;
        object2_isSphere = false;
	end

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    index = 21;
	r_db = [z(19:21)];
    v_db = [z(16:18)];
	r_sc = [z(index + 19:index + 21)];	
	v_sc = [z(index + 16:index + 18)];

	if (object1_isSphere == true) && (object2_isSphere == true)
        
		%% [m1 , a1, b1, c1] = object_1;
        %m1 = object_1(1);
        a1 = object_1(2);
        %b1 = object_1(3);
        %c1 = object_1(4);
		%[m2 , a2, b2, c2] = object_2;
        %% m2 = object_2(1);
        a2 = object_2(2);
        %b2 = object_2(3);
        %c2 = object_2(4);

		R_1 = a1;
		R_2 = a2;

		r_s2d = r_sc - r_db;
		v_s2d = v_sc - v_db;

		penetration = - norm(r_s2d) + (R_1 + R_2);
		penetration_rate = norm(v_s2d);

		if penetration > 0
			contact_status = 1;
		else 
			contact_status = 0;
		end

    elseif (object1_isSphere == true) && (object2_isSphere == false)

        % implement other geometry settings
		contact_status = 0;
        penetration = 0;
        penetration_rate =0;
        %---------------------------------%

	elseif (object1_isMembrane == true) && (object2_isSphere == true)
		contact_status = 0;
        penetration = 0;
        penetration_rate =0;
        %---------------------------------%
	else
		print('not implemented yet!');
    end
    contact_situation = [contact_status , penetration, penetration_rate];

end