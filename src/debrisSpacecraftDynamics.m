function debrisSpacecraftDynamics()
	
	% Set initial and final time
	t0= 0;
	tf= 10;
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

function dz = equations_of_motion(t,z)

	

	moment_of_inerta = get_moment_of_inertia('debris0000');

	I_11 = moment_of_inerta(1,1);
	I_22 = moment_of_inerta(2,2);
	I_33 = moment_of_inerta(3,3);

	omega_1 = z(1);
	omega_2 = z(2);
	omega_3 = z(3);

	moment = get_total_moment(t, 'debris0000');

	M_1 = moment(1);
	M_2 = moment(2);
	M_3 = moment(3);


	dz_rot_obj = zeros(6,1); % instantiate a zero vector for elements

	% I_11 * omega_dot_1 = M_1 + (I_22 - I_33) * omega_2 * omega_3
	dz_rot_obj(1,1) = (1/I_11) * (M_1 + (I_22 - I_33) * omega_2 * omega_3);
	% I_22 * omega_dot_2 = M_2 + (I_33 - I_11) * omega_3 * omega_1 
	dz_rot_obj(2,1) = (1/I_22) * (M_2 + (I_33 - I_11) * omega_3 * omega_1);
	% I_33 * omega_dot_3 = M_3 + (I_11 - I_22) * omega_1 * omega_2
	dz_rot_obj(3,1) = (1/I_33) * (M_3 + (I_11 - I_22) * omega_1 * omega_2);

	dz_rot_obj(4,1) = z(1,1);
	dz_rot_obj(5,1) = z(2,1);
	dz_rot_obj(6,1) = z(3,1);


	


	%%%%%%%%%%%%%%%%%%%%

	dz(1,1) = dz_rot_obj(1,1);
	dz(2,1) = dz_rot_obj(2,1);
	dz(3,1) = dz_rot_obj(3,1);
	dz(4,1) = dz_rot_obj(4,1);
	dz(5,1) = dz_rot_obj(5,1);
	dz(6,1) = dz_rot_obj(6,1);



	%%%%%%%%%%%%%%%%%%%%%%
	%    d/dt (rotation matrix) = omega skew symmetric * rotation matrix

	% it's not most efficient but it behaves well. better to use quaternians instead.

	%--------------------

    rotation_matrix = zeros(3,3);

    rotation_matrix(1,1) = z(7,1);
    rotation_matrix(1,2) = z(8,1);
    rotation_matrix(1,3) = z(9,1);
    rotation_matrix(2,1) = z(10,1);
    rotation_matrix(2,2) = z(11,1);
    rotation_matrix(2,3) = z(12,1);
    rotation_matrix(3,1) = z(13,1);
    rotation_matrix(3,2) = z(14,1);
    rotation_matrix(3,3) = z(15,1);

    if det(rotation_matrix)== 1
        test_determinant = 0;
    else
        test_determinant = det(rotation_matrix);
    end




	%rotation_matrix_dot = zeros(3,3);

	%rotation_matrix_dot(1,1) = - omega_3 * rotation_matrix(2,1) + omega_2 * rotation_matrix(3,1);
	%rotation_matrix_dot(1,2) = - omega_3 * rotation_matrix(2,2) + omega_2 * rotation_matrix(3,2); 
	%rotation_matrix_dot(1,3) = - omega_3 * rotation_matrix(2,3) + omega_2 * rotation_matrix(3,3); 

	% rotation_matrix_dot(2,1) = omega_3 * rotation_matrix(1,1) - omega_1 * rotation_matrix(3,1);
	% rotation_matrix_dot(2,2) = omega_3 * rotation_matrix(1,2) - omega_1 * rotation_matrix(3,2);
	% rotation_matrix_dot(2,3) = omega_3 * rotation_matrix(1,3) - omega_1 * rotation_matrix(3,3);

	% rotation_matrix_dot(3,1) = - omega_2 * rotation_matrix(1,1) + omega_1 * rotation_matrix(2,1); 
	% rotation_matrix_dot(3,2) = - omega_2 * rotation_matrix(1,2) + omega_1 * rotation_matrix(2,2); 
	% rotation_matrix_dot(3,3) = - omega_2 * rotation_matrix(1,3) + omega_1 * rotation_matrix(2,3); 

    rotation_matrix_dot =zeros(3,3);
    rotation_matrix_dot = [0 -omega_3 omega_2; omega_3 0 -omega_1; -omega_2 omega_1 0] * rotation_matrix;

    % test_matrix = rotation_matrix_dot_new - rotation_matrix_dot;
    % if test_matrix == zeros(3,3)
    %     hhh= 0;
    % else
    %     hhh = test_matrix
    % end




	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    dz(7,1) = rotation_matrix_dot(1,1);
    dz(8,1) = rotation_matrix_dot(1,2);
    dz(9,1) = rotation_matrix_dot(1,3);
    dz(10,1) = rotation_matrix_dot(2,1); 
    dz(11,1) = rotation_matrix_dot(2,2);
    dz(12,1) = rotation_matrix_dot(2,3);
    dz(13,1) = rotation_matrix_dot(3,1);
    dz(14,1) = rotation_matrix_dot(3,2);
    dz(15,1) = rotation_matrix_dot(3,3);

	%%%%%%%%%%%%%%%%%%%%%%
	[mass, a, b, c] = get_object_properties('spacecraft');

	external_force = get_total_force(t);

	dz(16,1) = (1/mass) * external_force(1);
    dz(17,1) = (1/mass) * external_force(2);
    dz(18,1) = (1/mass) * external_force(3);
    dz(19,1) = z(16,1);
	dz(20,1) = z(17,1);
	dz(21,1) = z(18,1);

	%--------------------------------------------------------------%
	%------------- End of Debris related states -------------------%
	%--------------------------------------------------------------%

	% Start priducing spacecraft related states
	% index is used to reset for used slots in dz
	index = 21; % 6+9+6 rotation + rotation matrix + translation

	moment_of_inerta_sc = get_moment_of_inertia('spacecraft');

	I_11 = moment_of_inerta_sc(1,1);
	I_22 = moment_of_inerta_sc(2,2);
	I_33 = moment_of_inerta_sc(3,3);

	omega_1 = z(index+1);
	omega_2 = z(index+2);
	omega_3 = z(index+3);

	moment = get_total_moment(t, 'spacecraft');

	M_1 = moment(1);
	M_2 = moment(2);
	M_3 = moment(3);


	dz_rot_sc = zeros(6,1); % instantiate a zero vector for elements

	% I_11 * omega_dot_1 = M_1 + (I_22 - I_33) * omega_2 * omega_3
	dz_rot_sc(1,1) = (1/I_11) * (M_1 + (I_22 - I_33) * omega_2 * omega_3);
	% I_22 * omega_dot_2 = M_2 + (I_33 - I_11) * omega_3 * omega_1 
	dz_rot_sc(2,1) = (1/I_22) * (M_2 + (I_33 - I_11) * omega_3 * omega_1);
	% I_33 * omega_dot_3 = M_3 + (I_11 - I_22) * omega_1 * omega_2
	dz_rot_sc(3,1) = (1/I_33) * (M_3 + (I_11 - I_22) * omega_1 * omega_2);

	dz_rot_sc(4,1) = z(index + 1,1);
	dz_rot_sc(5,1) = z(index + 2,1);
	dz_rot_sc(6,1) = z(index + 3,1);


	


	%%%%%%%%%%%%%%%%%%%%

	dz(index + 1,1) = dz_rot_obj(1,1);
	dz(index + 2,1) = dz_rot_obj(2,1);
	dz(index + 3,1) = dz_rot_obj(3,1);
	dz(index + 4,1) = dz_rot_obj(4,1);
	dz(index + 5,1) = dz_rot_obj(5,1);
	dz(index + 6,1) = dz_rot_obj(6,1);


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	rotation_matrix = zeros(3,3);

    rotation_matrix(1,1) = z(index + 7,1);
    rotation_matrix(1,2) = z(index + 8,1);
    rotation_matrix(1,3) = z(index + 9,1);
    rotation_matrix(2,1) = z(index + 10,1);
    rotation_matrix(2,2) = z(index + 11,1);
    rotation_matrix(2,3) = z(index + 12,1);
    rotation_matrix(3,1) = z(index + 13,1);
    rotation_matrix(3,2) = z(index + 14,1);
    rotation_matrix(3,3) = z(index + 15,1);

    if det(rotation_matrix)== 1
        test_determinant = 0;
    else
        test_determinant = det(rotation_matrix);
    end




	%rotation_matrix_dot = zeros(3,3);

	%rotation_matrix_dot(1,1) = - omega_3 * rotation_matrix(2,1) + omega_2 * rotation_matrix(3,1);
	%rotation_matrix_dot(1,2) = - omega_3 * rotation_matrix(2,2) + omega_2 * rotation_matrix(3,2); 
	%rotation_matrix_dot(1,3) = - omega_3 * rotation_matrix(2,3) + omega_2 * rotation_matrix(3,3); 

	% rotation_matrix_dot(2,1) = omega_3 * rotation_matrix(1,1) - omega_1 * rotation_matrix(3,1);
	% rotation_matrix_dot(2,2) = omega_3 * rotation_matrix(1,2) - omega_1 * rotation_matrix(3,2);
	% rotation_matrix_dot(2,3) = omega_3 * rotation_matrix(1,3) - omega_1 * rotation_matrix(3,3);

	% rotation_matrix_dot(3,1) = - omega_2 * rotation_matrix(1,1) + omega_1 * rotation_matrix(2,1); 
	% rotation_matrix_dot(3,2) = - omega_2 * rotation_matrix(1,2) + omega_1 * rotation_matrix(2,2); 
	% rotation_matrix_dot(3,3) = - omega_2 * rotation_matrix(1,3) + omega_1 * rotation_matrix(2,3); 

    rotation_matrix_dot =zeros(3,3);
    rotation_matrix_dot = [0 -omega_3 omega_2; omega_3 0 -omega_1; -omega_2 omega_1 0] * rotation_matrix;

    % test_matrix = rotation_matrix_dot_new - rotation_matrix_dot;
    % if test_matrix == zeros(3,3)
    %     hhh= 0;
    % else
    %     hhh = test_matrix
    % end




	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    dz(index + 7,1) = rotation_matrix_dot(1,1);
    dz(index + 8,1) = rotation_matrix_dot(1,2);
    dz(index + 9,1) = rotation_matrix_dot(1,3);
    dz(index + 10,1) = rotation_matrix_dot(2,1); 
    dz(index + 11,1) = rotation_matrix_dot(2,2);
    dz(index + 12,1) = rotation_matrix_dot(2,3);
    dz(index + 13,1) = rotation_matrix_dot(3,1);
    dz(index + 14,1) = rotation_matrix_dot(3,2);
    dz(index + 15,1) = rotation_matrix_dot(3,3);

	%%%%%%%%%%%%%%%%%%%%%%
	[mass, a, b, c] = get_object_properties('spacecraft');

	external_force = get_total_force(t);

	dz(index + 16,1) = (1/mass) * external_force(1);
    dz(index + 17,1) = (1/mass) * external_force(2);
    dz(index + 18,1) = (1/mass) * external_force(3);
    dz(index + 19,1) = z(index + 16,1);
	dz(index + 20,1) = z(index + 17,1);
	dz(index + 21,1) = z(index + 18,1);

end

function get_contact_force()

	if contact_flag==1
	    %--------------
		r= (R_1^-1+R_2^-1)^-1;
		h_1= (1-nou_1^2)/(pi*E_1);
		h_2= (1-nou_2^2)/(pi*E_2);
		k_c= (4/(3*pi))*(sqrt(r)/(h_1+h_2));
		lambda = 1.5*alpha*k_c;
		contact_force = k_c*delta^nn+lambda*delta^nn*delta_dot; 
		else
		contact_force = 0; 
	    X_i = 0;
	    Y_i = 0;
	end

	force =[contact_force,X_i,Y_i];

end

function contact_status = detect_contact(t)

	contact_status = 0;

end

function m = get_total_moment(t, object)

	if object == 'debris0000'
		m = [0;0;0];
	elseif object == 'spacecraft'
		m = [0;0;0];
	else
		print('Object is unknown!');
	end

end
function f = get_total_force(t)

	f = [0.4;0;0];

end
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


function initial_conditions = get_initial_conditions()

	initial_orientation = [0;0;0];
	initial_euler_angles = [1;2;3]; % [si, theta, phi]
	initial_rates_B = [1;1;0.5]; % rad/s - w_body represented in body frame B

	%initial_rotation_matrix = [1 0 0; 0 1 0; 0 0 1]; % identity for test0, care for reshape usage!!!!!
	initial_rotation_matrix = euler_angles2rotation_matrix(initial_euler_angles);
	%initial_rates_R = initial_rotation_matrix * initial_rates_B;

	initial_position = [0;0;0];
	initial_velocity = [1;0;1];

	initial_conditions_db = [initial_rates_B; initial_orientation; reshape(transpose(initial_rotation_matrix),[9,1]); initial_velocity; initial_position];

	initial_orientation_sc = [0;0;0];
	initial_euler_angles_sc = [0;0;0]; % [si, theta, phi]
	initial_rates_Sc2R = [1;1;0.5]; % rad/s - w_body represented in body frame B

	%initial_rotation_matrix = [1 0 0; 0 1 0; 0 0 1]; % identity for test0, care for reshape usage!!!!!
	initial_rotation_matrix_Sc2R = euler_angles2rotation_matrix(initial_euler_angles_sc);
	%initial_rates_R = initial_rotation_matrix * initial_rates_B;

	initial_position_Sc = [1;1;1];
	initial_velocity_Sc = [0;-1;0];

	initial_conditions_sc = [initial_rates_Sc2R; initial_orientation_sc; reshape(transpose(initial_rotation_matrix_Sc2R),[9,1]); initial_velocity_Sc; initial_position_Sc];
	initial_conditions = [initial_conditions_db;initial_conditions_sc];

end

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

function visualize_motion(t1,t2)
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
	for j=1:length(t)
		figure(2)
		hold off
		%[X,Y,Z] = ellipsoid(xc,yc,zc,xr,yr,zr)
		[m, a, b, c] = get_object_properties('debris0000');





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

    	rotation_matrix_R2B = transpose(rotation_matrix);

		[amount_rotation, direction] = rotation_matrix2euler_angleAxis(rotation_matrix_R2B);
		if amount_rotation == 0 
			direction = [1; 0; 0]; % dummy direction
		end

		amount_rotation_in_deg = amount_rotation * (180/pi);
		%direction = [e1 e2 e3];


		x_c = z(j,19);
		y_c = z(j,20);
		z_c = z(j,21);

		%center_position_vector = rotation_matrix * [x_ce; y_ce; z_ce];
		%x_c = center_position_vector(1);
		%y_c = center_position_vector(2);
		%z_c = center_position_vector(3);
		[X,Y,Z] = ellipsoid(0, 0, 0, a, b, c);
		s = surf(X + x_c, Y + y_c, Z + z_c);
		rotate(s,direction,amount_rotation_in_deg)
		title(num2str(round(100*t(j))/100));
		% shading interp
    	view([0 0 1]);
    	xlim([-5 5]);
    	ylim([-5 5]);
    	zlim ([-5 5]);
    	xlabel('X (m)')
    	ylabel('Y (m)')
    	zlabel('Z (m)')
		hold on
		%pause(dt)
	end
end

function [theta, e_hat] = rotation_matrix2euler_angleAxis(rotation_matrix)
	% in this function we implement the transform from a rotation matrix to 
	% a single axis (Euller Axis) and an angle of rotation to go from frame A to fram B
	% Eigendecomposition of the rotation matrix yields the eigenvalues 1 and cos θ ± i sin θ. 
	% The Euler axis is the eigenvector corresponding to the eigenvalue of 1, 
	% and θ can be computed from the remaining eigenvalues.
	% The Euler axis can be also found using singular value decomposition
	% since it is the normalized vector spanning the null-space
	% of the matrix I − A.

	%  R_A2B ---> e_hat = [e1;e2;e3] and theta

	theta = acos((1/2)*(trace(rotation_matrix)-1));    % amount of rotation in radians 
	e1 = (rotation_matrix(3,2) - rotation_matrix(2,3))/(2 * sin(theta)); % 1st element of Euler Axis
	e2 = (rotation_matrix(1,3) - rotation_matrix(3,1))/(2 * sin(theta));	% 2nd element of Euler Axis
	e3 = (rotation_matrix(2,1) - rotation_matrix(1,2))/(2 * sin(theta));	% 3rd element of Euler Axis
	e_hat = [e1;e2;e3];

end


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


function rotation_matrix = euler_angles2rotation_matrix(euler_angles)

	%eul = [si theta phi];
	eul = transpose(euler_angles);
	rotation_matrix = eul2rotm(eul); % default is "ZYX" sequence

end

function euler_angles = rotation_matrix2euler_angles(rotation_matrix)

	%eul = [si theta phi];
	%eul = transpose(euler_angles);
	%rotation_matrix = eul2rotm(eul); % default is "ZYX" sequence

	euler_angles = rotm2eul(rotation_matrix);

end

function control_force = get_control_force()

	control_force = [0;0;0];
end

function control_moment = get_control_moment()

	control_moment = [0;0;0];
end