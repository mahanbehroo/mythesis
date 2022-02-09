function f = get_total_force(t, object1, object2, z)
    
    contact_situation = detect_contact(t, z);

    contact_status = contact_situation(1);
    penetration = contact_situation(2);
    penetration_rate = contact_situation(3);

	if object1 == 'spacecraft'
		if contact_status == 0
			f = get_control_force(t, 'spacecraft');
		else
			f = get_contact_force('debris0000', 'spacecraft', contact_situation) + get_control_force(t, 'spacecraft');
		end
    else object1 == 'debris0000'
        if contact_status == 0
            f = [0;0;0];
        else
            f = get_contact_force('debris0000', 'spacecraft', contact_situation);
        end
    end
end