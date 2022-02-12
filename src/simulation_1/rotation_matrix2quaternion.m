function quaternion = rotation_matrix2quaternion(rotation_matrix)

	%tr = trace(rotation_matrix);
	r = rotation_matrix;
	quaternion = zeros(4,1);
	%quaternion(1) = sqrt(1 + tr) / 2;
	%quaternion(2) = (r(3,2) - r(2,3)) / (4 * quaternion(1));
	%quaternion(3) = (r(1,3) - r(3,1)) / (4 * quaternion(1));
	%quaternion(4) = (r(2,1) - r(1,2)) / (4 * quaternion(1));
	%z = norm(quaternion);
	%quaternion = quaternion / z;



	
	quaternion(1) = sqrt( max( 0, 1 + r(1,1) + r(2,2) + r(3,3) ) ) / 2;
	quaternion(2) = sqrt( max( 0, 1 + r(1,1) - r(2,2) - r(3,3) ) ) / 2;
	quaternion(3) = sqrt( max( 0, 1 - r(1,1) + r(2,2) - r(3,3) ) ) / 2;
	quaternion(4) = sqrt( max( 0, 1 - r(1,1) - r(2,2) + r(3,3) ) ) / 2;
	%Q.x = _copysign( quaternion(2), r(3,2) - r(2,3) );
	quaternion(2) = sign(r(3,2) - r(2,3)) * abs(quaternion(2));


	%Q.y = _copysign( quaternion(3), r(1,3) - r(3,1) );
	quaternion(3) = sign(r(1,3) - r(3,1)) * abs(quaternion(3));
	%Q.z = _copysign( quaternion(4), r(2,1) - r(1,2) );
	quaternion(4) = sign(r(2,1) - r(1,2)) * abs(quaternion(4));

end