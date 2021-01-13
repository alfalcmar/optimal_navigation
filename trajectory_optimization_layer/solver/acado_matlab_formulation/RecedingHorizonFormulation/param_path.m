switch path
    case 0
        user_points = 5;
        time_interval = 20;
        steps         = 50;
        steps_rec_hor = 22;

        points = [    5,     5; ...
                      8,     6; ...
                     22,    18; ...
                      7,  23.5; ...
                    3.5,   4.5];
        
    case 1
        user_points = 4;
        time_interval = 20;
        steps         = 50;
        steps_rec_hor = 22;
        
        points = [    5,     5; ...
                     22,    18; ...
                      9,  24.5; ...
                      3,     9];
        
    case 2
        user_points = 3;
        time_interval = 25;
        steps         = 100;
        steps_rec_hor = 41;

        points = [    5,     5; ...
                     25,    20; ...
                     12,    35];
        
    case 3
        user_points = 4;
        time_interval = 50;
        steps         = 120;
        steps_rec_hor = 65;

        points = [    5,     5; ...
                     50,    60; ...
                     90,   110; ...
                    140,   120];
        
    otherwise
        disp ('Not have chosen a path between 0 or 3')
end

n_points    = user_points; % For plots
