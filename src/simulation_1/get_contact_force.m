function contact_force = get_contact_force(object1, object2, contact_situation)

	
	if object == 'spacecraft'


		if contact_status==1
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

		contact_force =[contact_force,X_i,Y_i];

	else
		contact_force =[0,0,0];

end