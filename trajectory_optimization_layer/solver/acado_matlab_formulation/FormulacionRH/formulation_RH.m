function out = formulation_RH(ps_drone, pe_drone, t_start, t_end, time_horizon_, px_windmill, py_windmill, r_windmill, v_initial, a_initial, a_final)

% ---------------------------- ACADO'S CODE ---------------------------- %

% Always start with "BEGIN_ACADO". 
BEGIN_ACADO;

% Set the ACADO problem name:
acadoSet('problemname', 'one_drone_simple_trajectory'); 

% % Windmill's position and radius (no-fly zone) [m]:
% px_windmill = 12.0;
% py_windmill = 16.0;
% r_windmill  = 9.0;

% % Drone's start and end pose [x, y, z] [m]:
% ps_drone     = [-10.0, -15.0, 0.0];
% pe_drone     = [30.0, 45.0, 15.0];

% Max drone's velocity and acceleration
MAX_ACC    = 5.0;
MAX_VEL_XY = 15.0;
MAX_VEL_Z  = 15.0;

% % Define t_start, t_end and time_horizon (interval's step size)
% t_start = 0.0;
% t_end = 30.0;
% time_horizon_ = 100;

% Drone's pose and velocity
DifferentialState   px_ py_ pz_ vx_ vy_ vz_

% Drone's accelerations
Control ax_ ay_ az_

% Drone's dynamic model
model = acado.DifferentialEquation();

% In C++, use VariablesGrid
% my_grid_ = zeros(1,3);
% my_grid_ = [t_start,t_end,time_horizon_];

% Define the kinematic model
model.add(dot(px_) == vx_);
model.add(dot(py_) == vy_);
model.add(dot(pz_) == vz_);
model.add(dot(vx_) == ax_);
model.add(dot(vy_) == ay_);
model.add(dot(vz_) == az_);
    
% Define the formulation
ocp = acado.OCP(t_start, t_end, time_horizon_);

% Use de kinematic model on the optimization control problem
ocp.subjectTo( model );

% Constraints (define MAX_ACC and MAX_VEL as you wish)
ocp.subjectTo(  -MAX_ACC    <= ax_ <= MAX_ACC   );  
ocp.subjectTo(  -MAX_ACC    <= ay_ <= MAX_ACC   );
ocp.subjectTo(  -MAX_ACC    <= az_ <= MAX_ACC   );
ocp.subjectTo(  -MAX_VEL_XY <= vx_ <= MAX_VEL_XY   );
ocp.subjectTo(  -MAX_VEL_XY <= vy_ <= MAX_VEL_XY   );
ocp.subjectTo(  -MAX_VEL_Z  <= vz_ <= MAX_VEL_Z   );
ocp.subjectTo(  -MAX_VEL_Z  <= vz_ <= MAX_VEL_Z   );

% Initial state
ocp.subjectTo( 'AT_START', px_ == ps_drone(1));
ocp.subjectTo( 'AT_START', py_ == ps_drone(2));
ocp.subjectTo( 'AT_START', pz_ == ps_drone(3));
ocp.subjectTo( 'AT_START', vx_ == v_initial(1));
ocp.subjectTo( 'AT_START', vy_ == v_initial(2));
ocp.subjectTo( 'AT_START', vz_ == v_initial(3));
ocp.subjectTo( 'AT_START', ax_ == a_initial(1));
ocp.subjectTo( 'AT_START', ay_ == a_initial(2));
ocp.subjectTo( 'AT_START', az_ == a_initial(3));

% Consider the restriction of the no-fly zone (windmill)
ocp.subjectTo(  (px_ - px_windmill)^2 + (py_ - py_windmill)^2 >= (r_windmill)^2);

% Function h_1;
h_1 = [px_, py_, pz_];

% Set weights as you wish
S_1      = eye(3, 3);
S_1(1,1) = 1;   %W_PX_N;
S_1(2,2) = 1;   %W_PY_N;
S_1(3,3) = 1;   %W_PZ_N;

% Set desired pose
r_1 (1:3) = pe_drone; 

ocp.minimizeLSQEndTerm( S_1, h_1, r_1 );

% Function h;
h = [ax_ ay_ az_];

% Weights
S = eye(3, 3);
S(1,1) = 1; %W_AX;
S(2,2) = 1; %W_AY;
S(3,3) = 1; %W_AZ;

% At the final point, use the theoretical acceleration to reach that point
r = [a_final(1), a_final(2), a_final(3)];

ocp.minimizeLSQ( S, h, r );


% Set up the optimization algorithm
algo = acado.OptimizationAlgorithm(ocp);

% Set some parameters for the algorithm (can be commented)
algo.set('KKT_TOLERANCE', 1e-4 );
algo.set('INTEGRATOR_TOLERANCE', 1e-1 );
algo.set('RELAXATION_PARAMETER', 3.5 );

% Always end with "END_ACADO".
END_ACADO;

% -------------------------- END ACADO'S CODE -------------------------- %

% Now, let's run the test!
out = one_drone_simple_trajectory_RUN();

% Once it compiles, we check the output states and control
% C++:
% solver.getDifferentialStates(output_states);
% solver.getControls          (output_control);

% MATLAB:
% draw;

end