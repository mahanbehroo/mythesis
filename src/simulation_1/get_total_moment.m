function m = get_total_moment(t, object, force, lever)

	if object == 'debris0000'
		m = cross(lever, force);
	elseif object == 'spacecraft'
		m = cross(lever, force);
	else
		print('Object is unknown!');
	end

end
