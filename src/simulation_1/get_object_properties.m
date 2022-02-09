function object_properties = get_object_properties(object)

	if object == 'debris0000'
		mass = 20; % kg
		a = 3; % m
		b = 3; % m 
		c = 3; % m
	elseif object == 'spacecraft'
		mass = 40; % kg
		a = 1; % m
		b = 1; % m 
		c = 1; % m

	else
		
		print('Object is unknown to me!');
	end
    object_properties = [mass, a, b, c];
end