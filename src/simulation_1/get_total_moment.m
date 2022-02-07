function m = get_total_moment(t, object)

	if object == 'debris0000'
		m = [0;0;0];
	elseif object == 'spacecraft'
		m = [0;0;0];
	else
		print('Object is unknown!');
	end

end
