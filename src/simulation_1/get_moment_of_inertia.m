function moi = get_moment_of_inertia(object)

	if object == 'debris0000'

    	[mass, a, b, c] = get_object_properties('debris0000');

    	moi = (3/5) * mass * [b+c 0 0; 0 c+a 0; 0 0 a+b];


    elseif object == 'spacecraft'

    	[mass, a, b, c] = get_object_properties('spacecraft');

    	moi = (3/5) * mass * [b+c 0 0; 0 c+a 0; 0 0 a+b];


    else 
    	print("I can't understand the onject!");
    end
    
end
