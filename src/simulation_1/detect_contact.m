function contact_situation = detect_contact(t, object1, object2, z)

	object_1 = get_object_properties(object1);
	object_2 = get_object_properties(object2);

	if object_1(2) == object_1(3) && object_1(2) = object_1(4)
		object1_geometry = 'sphere';
	else
		object1_geometry = 'ellipsoid';
	end

	if object_2(2) == object_2(3) && object_2(2) = object_2(4)
		object1_geometry = 'sphere';
	else
		object2_geometry = 'ellipsoid';
	end

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	if object1_geometry == 'sphere' && object2_geometry == 'sphere'
		r_db = [];
		r_sc = [];
		v_db = [];
		v_sc = [];

		[m1 , a1, b1, c1] = object_1;
		[m2 , a2, b2, c2] = object_2;

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

	elseif object1 == 'sphere' && object2 == 'membrane'
		contact_status = 0;
	else
		print('not implemented yet!');
	end

	contact_situation = [contact_status , penetration, penetration_rate];

end