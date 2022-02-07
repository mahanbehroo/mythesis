function plot_results()
	tic
	%----------------- Reading the simulation output file --------------------%
	load 'Outputs\\OutputFile.txt'
	fprintf('\nElapsed time loading data file (sec) = %1.3f\n',toc)
	%----------------- plotting m_ij's position vs time ----------------------%
	T	= OutputFile(:,1);
	Z   = OutputFile(:,2:end);
	t0 = OutputFile(1,1);
	tf = OutputFile(end,1);

	dt = (tf-t0)/100;
	t= (t0:dt:tf)';
	z = interp1(T,Z,t);

	index = 21;

    for j=1:length(t)
	    rotation_matrix = zeros(3,3);
    
	    rotation_matrix(1,1) = z(j,7);
	    rotation_matrix(1,2) = z(j,8);
	    rotation_matrix(1,3) = z(j,9);
	    rotation_matrix(2,1) = z(j,10);
	    rotation_matrix(2,2) = z(j,11);
	    rotation_matrix(2,3) = z(j,12);
	    rotation_matrix(3,1) = z(j,13);
	    rotation_matrix(3,2) = z(j,14);
	    rotation_matrix(3,3) = z(j,15);
    
	    euler_angles(j, 1:3) = rotation_matrix2euler_angles(rotation_matrix);

	    rotation_matrix_Sc2R = zeros(3,3);
    
	    rotation_matrix_Sc2R(1,1) = z(j,7);
	    rotation_matrix_Sc2R(1,2) = z(j,8);
	    rotation_matrix_Sc2R(1,3) = z(j,9);
	    rotation_matrix_Sc2R(2,1) = z(j,10);
	    rotation_matrix_Sc2R(2,2) = z(j,11);
	    rotation_matrix_Sc2R(2,3) = z(j,12);
	    rotation_matrix_Sc2R(3,1) = z(j,13);
	    rotation_matrix_Sc2R(3,2) = z(j,14);
	    rotation_matrix_Sc2R(3,3) = z(j,15);
    
	    euler_angles_sc(j, 1:3) = rotation_matrix2euler_angles(rotation_matrix_Sc2R);
    end

	figure(1)
	subplot(2,2,1)
	plot(t,z(:,1:3),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Object Angular Velocity Components')
	subplot(2,2,2)
	plot(t,z(:,4:6),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Object Angular Position Components')

	subplot(2,2,3)
	plot(t,z(:,7:15),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Rotation Matrix Components');

	subplot(2,2,4)
    plot(t, euler_angles(:, 1:3),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Euler Angles ZYX (rad)')


	figure(3)
	subplot(2,2,1)
	plot(t,z(:,16:18),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Object Velocity Components')
	subplot(2,2,2)
	plot(t,z(:,19:21),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Object Position Components')



	figure(4)
	subplot(2,2,1)
	plot(t,z(:,index + 1:index + 3),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Spacecraft Angular Velocity Components')
	subplot(2,2,2)
	plot(t,z(:,index + 4:index + 6),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Spacecraft Angular Position Components')

	subplot(2,2,3)
	plot(t,z(:,index + 7:index + 15),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Sc2R Rotation Matrix Components');

	subplot(2,2,4)
    plot(t, euler_angles_sc(:, 1:3),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Spacecraft Euler Angles ZYX (rad)')


	figure(5)
	subplot(2,2,1)
	plot(t,z(:,index + 16:index + 18),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Spacecraft Velocity Components')
	subplot(2,2,2)
	plot(t,z(:,index + 19:index + 21),'LineWidth',2);
	xlabel('Time (sec)')
	ylabel('Spacecraft Position Components')

end