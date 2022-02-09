function moi = get_moment_of_inertia(object)

	if object == 'debris0000'

    	object_properties = get_object_properties('debris0000');

        mass = object_properties(1);
        a = object_properties(2);
        b = object_properties(3);
        c = object_properties(4);

    	moi = (3/5) * mass * [b+c 0 0; 0 c+a 0; 0 0 a+b];


    elseif object == 'spacecraft'

    	object_properties = get_object_properties('spacecraft');
        mass = object_properties(1);
        a = object_properties(2);
        b = object_properties(3);
        c = object_properties(4);

    	moi = (3/5) * mass * [b+c 0 0; 0 c+a 0; 0 0 a+b];


    else 
    	print("I can't understand the onject!");
    end
    
end
