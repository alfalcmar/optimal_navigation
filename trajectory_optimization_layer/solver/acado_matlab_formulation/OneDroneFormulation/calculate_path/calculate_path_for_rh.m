function path = calculate_path_for_rh (R, NO_FLY_ZONE_XY, next_points, steps)

npoints = size(next_points, 1);

for i = 1 : (npoints-1)
    path_aux = calculate_path(R, NO_FLY_ZONE_XY, next_points(i,:), next_points(i+1,:), steps);
    path((i-1)*steps + 1 : i*steps, :) = path_aux;
end

end