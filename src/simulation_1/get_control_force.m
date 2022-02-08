function control_force = get_control_force(t,object)
	if object == 'spacecraft'
		% code control scheme here!
		control_force = [0;0;0];
		% -------------------------
	else
		print('You dont have thruster! sorry!');
	end
end
