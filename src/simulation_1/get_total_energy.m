function  totalEnergy = get_total_energy(t, z)
	% add energy equations to sum up the energy of whole system
	% this function is meant to check numerical sanity of code 

	% get parameters ready...



	mass_spacecraft = ;
	moi_spacecraft = get_moment_of_inertia('spacecraft');
	mass_debris = ;
	moi_debris = get_moment_of_inertia('debris0000');
	velocity_spacecraft = z();
	omega_spacecraft = z();
	velocity_debris = z();
	omega_debris = z();

	%----------------------------------------------------------
	% center of mass related contribution

	translational_kinetic_spacecraft = (1/2) * mass_spacecraft * (norm(velocity_spacecraft))^2;

	rotational_kinetic_spacecraft = (1/2) * moi_spacecraft * (norm(omega_spacecraft))^2;

	translational_kinetic_debris = (1/2) * mass_debris * (norm(velocity_debris))^2;

	rotational_kinetic_debris = (1/2) * moi_debris * (norm(omega_debris))^2;


	kinetic_spacecraft = translational_kinetic_spacecraft + rotational_kinetic_spacecraft;

	kinetic_debris = translational_kinetic_debris + rotational_kinetic_debris;

	%------------------------------------------------------------------------------


	% vibration related contribution

	membrane_energy = membrane_kinetic + membrane_potential; 
 	
 	%--------------------------------------------------------------------


	totalE_spacecraft = kinetic_spacecraft + potential_spacecraft + membrane_energy ;
	total_E_debris = kinetic_debris + potential_debris;
	totalEnergy = totalE_spacecraft + total_E_debris;

end