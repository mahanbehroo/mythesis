function object_properties = get_object_properties(object)
	shape = 'membrane0'; % temporary

	if object == 'debris0000'
		mass = 20; % kg
		a = 3; % m
		b = 3; % m 
		c = 3; % m
	elseif object == 'spacecraft'
		if shape == 'ellipsoid'
			mass = 40; % kg
			a = 2; % m
			b = 2; % m 
			c = 2; % m
		elseif shape == 'membrane0'
		    rho = 1;
			L = 1;
			mass = rho * L * L;
			a = L;
			b = L;
			c = 0;
		else 
			print('spacecraft shape is undefined');
		end
	else
		
		print('Object is unknown to me!');
	end
    object_properties = [mass, a, b, c];
end