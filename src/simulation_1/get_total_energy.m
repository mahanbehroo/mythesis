function  totalEnergy = get_total_energy(t, z)
	% add energy equations to sum up the energy of whole system
	% this function is meant to check numerical sanity of code 

	totalE_spacecraft = kinetic_spacecraft + potential_spacecraft ;
	total_E_debris = kinetic_debris + potential_debris;
	totalEnergy = totalE_spacecraft + total_E_debris;

end