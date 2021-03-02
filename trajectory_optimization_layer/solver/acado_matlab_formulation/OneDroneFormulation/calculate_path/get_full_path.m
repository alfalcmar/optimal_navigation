function path = get_full_path(r_windmill, px_windmill, py_windmill, points, steps_path, user_points)

for i = 1 : (user_points-1)
    path_aux = calculate_path(r_windmill, [px_windmill, py_windmill], points(i,:), points(i+1,:), steps_path);
    path((i-1)*steps_path + 1 : i*steps_path, :) = path_aux;
end