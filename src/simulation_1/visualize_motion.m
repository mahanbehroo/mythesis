
function visualize_motion(t1,t2)
	tic
	%----------------- Reading the simulation output file --------------------%
	OutputFile = load ('Outputs\\OutputFile.txt');
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
	reset_index = 50; 
	numebrOfModes = 7;
	L = 1;

	[X1,Y1] = meshgrid(0:0.05:L,0:0.05:L);
	dt1 = (tf-t0)/length(X1);
    t_mem= (t0:dt1:tf)';

	for j=1:length(t)
		figure(2)
		subplot(2,2,1)
		hold off
		%[X,Y,Z] = ellipsoid(xc,yc,zc,xr,yr,zr)
		object_properties = get_object_properties('debris0000');
        a = object_properties(2);
        b = object_properties(3);
        c = object_properties(4);

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
        origin = [x_c, y_c, z_c];
		rotate(s,direction,amount_rotation_in_deg, origin)

		hold on

		%------------------------------------------------------------%
		% Visualizing spacecraft
		
		object_properties = get_object_properties('spacecraft');
        a = object_properties(2);
        b = object_properties(3);
        c = object_properties(4);

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

		%[X1,Y1,Z1] = ellipsoid(0, 0, 0, a, b, c);

		zz =zeros (length(X1),length(Y1));
	    %element = zeros (length(x),1);
	    etha_temp = zeros (length(X1),length(X1));
	    for i=1:numebrOfModes
	        tau_x(j,i) = z(j, reset_index + i);
	        tau_y(j,i) = z(j,reset_index + 2 * numebrOfModes + i );
	        phi_x(:,i) = sin((i*pi/L)*X1(1,:));
	        phi_y(:,i) = sin((i*pi/L)*Y1(:,1));
	        etha_matrix = tau_x(j,i) * (phi_x(:,i) * (phi_y(:,i))');
	        
	        etha_temp = etha_temp + etha_matrix;
	    end
	   % for i=1:n
	    %    element(:) = etha(:,i)+ element(:);
	   % end
	    for i=1:length(X1)
	        for k=1:length(Y1)
	            zz(i,k)= zz(i,k)+etha_temp(i,k);
	            
	        end
        end


       	

		%s1 = surf(X1 + x_c, Y1 + y_c , zz + z_c);
		%origin = [x_c, y_c, z_c];
        %rotate(s1, direction_1, amount_rotation_in_deg, origin)
		%---------------------------------------------------------------------



		title(num2str(round(100*t(j))/100));
		% shading interp
    	view([1 1 1]);
    	xlim([-10 5]);
    	ylim([-10 5]);
    	zlim ([-5 5]);
    	xlabel('X (m)')
    	ylabel('Y (m)')
    	zlabel('Z (m)')
		hold on
		%pause(dt)


		figure(2)
		hold off
		subplot(2,2,2)

		s2 = surf(X1, Y1, zz)
		view([1 1 1]);
    	%xlim([-10 5]);
    	ylim([0 L]);
    	zlim ([-L/50 L/50]);
    	%xlabel('X (m)')
    	ylabel('Y (m)')
    	zlabel('Z (m)')
    	%hold on
	end
end
