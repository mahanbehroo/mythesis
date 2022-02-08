
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

	index =21;

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

		hold on

		%------------------------------------------------------------%
		% Visualizing spacecraft
		
		[m, a, b, c] = get_object_properties('spacecraft');

		rotation_matrix = zeros(3,3);

    	rotation_matrix(1,1) = z(j, index + 7);
    	rotation_matrix(1,2) = z(j, index + 8);
    	rotation_matrix(1,3) = z(j, index + 9);
    	rotation_matrix(2,1) = z(j, index + 10);
    	rotation_matrix(2,2) = z(j, index + 11);
    	rotation_matrix(2,3) = z(j, index + 12);
    	rotation_matrix(3,1) = z(j, index + 13);
    	rotation_matrix(3,2) = z(j, index + 14);
    	rotation_matrix(3,3) = z(j, index + 15);

    	rotation_matrix_R2S = transpose(rotation_matrix);
    	[amount_rotation, direction_1] = rotation_matrix2euler_angleAxis(rotation_matrix_R2S);

    	if amount_rotation == 0 
			direction_1 = [1; 0; 0]; % dummy direction
		end

		amount_rotation_in_deg = amount_rotation * (180/pi);
		%direction = [e1 e2 e3];

		x_c = z(j, index + 19);
		y_c = z(j, index + 20);
		z_c = z(j, index + 21);

		[X1,Y1,Z1] = ellipsoid(0, 0, 0, a, b, c);
		s1 = surf(X1, Y1 , Z1);
		rotate(s1, direction_1, amount_rotation_in_deg)
		%---------------------------------------------------------------------



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
