function [mass, a, b, c] = get_object_properties(object)

	if object == 'debris0000'
		mass = 20; % kg
		a = 1; % m
		b = 1; % m 
		c = 5; % m
	elseif object == 'spacecraft'
		mass = 40; % kg
		a = 4; % m
		b = 4; % m 
		c = 4; % m

	else
		
		print('Object is unknown to me!');
	end

end