function debrisSpacecraftDynamics()
	
	% Set initial and final time
	t0= 0;
	tf= 3;
	%----------------------------%
	% Set initial conditions for ODE solver
	initial_conditions = get_initial_conditions();
	%-----------------------------------------------%
	% Set ODE absolute and relative tolerances
	options = odeset('RelTol',1e-12,'AbsTol',1e-12*ones(length(initial_conditions),1));
	%----------------------------------------------------------------------------------%
	% Start integrating the state variables
	tic
	[T,Z] = ode45(@equations_of_motion,[t0 tf],initial_conditions,options);
	fprintf('\nElapsed time solving ODEs (sec) = %1.3f\n',toc)
	% ODE solved -------------------------------------------------------%
	%==================================================================================%
	% Saving [T,Z] in file
	tic
	st_name	= ['Outputs\\OutputFile.txt'];
	file_h	= fopen(st_name,'w');
	temp_for_saving_data = [T,Z];
	for i_print=1:size(temp_for_saving_data,1)
		for j_print=1:size(temp_for_saving_data,2)
		fprintf(file_h,'%20.10e ',temp_for_saving_data(i_print,j_print));
		end
	fprintf(file_h,'\n');
	end
	fclose(file_h);
	fprintf('\nElapsed time saving data to file (sec) = %1.3f\n',toc)
	%=================================================================================%
	%---------------------------------------------------------------------------------%
	% Getting plots and animations =========================%

	plot_results()

	visualize_motion()

	%===============================%
end

