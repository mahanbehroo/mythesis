function f = get_total_force(t, object)
    contact_status = detect_contact();
	if object == 'spacecraft'
		if contact_status == 0
			f = get_control_force(t, 'spacecraft');
		else
			f = get_contact_force('spacecraft') + get_control_force(t, 'spacecraft');
		end
    else object == 'debris0000'
        if contact_status == 0
            f = [0;0;0];
        else
            f = get_contact_force('debris0000')
        end
    end
end