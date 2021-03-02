function out = formulation(ps_drone, t_start, t_end, time_horizon_, next_points, v_initial, a_initial)

% ---------------------------- ACADO'S CODE ----------------------------

% Always start with "BEGIN_ACADO". 
BEGIN_ACADO;

% Set the ACADO problem name:
acadoSet('problemname', 'one_drone_simple_trajectory'); 

% Max drone's velocity and acceleration
MAX_ACC     = 1;
MAX_VEL_XY  = 3;
MAX_VEL_Z   = 3;

% Drone's pose and velocity
DifferentialState   px_ py_ pz_ vx_ vy_ vz_

% Drone's accelerations
Control ax_ ay_ az_

% Drone's dynamic model
model = acado.DifferentialEquation();

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

% % Consider the restriction of the no-fly zone (windmill)
% ocp.subjectTo(  (px_ - px_windmill)^2 + (py_ - py_windmill)^2 >= (r_windmill)^2);

h_1 = [px_, py_, pz_];

% Set weights as you wish
S_1      = eye(3, 3);
S_1(1,1) = 1;   %W_PX_N;
S_1(2,2) = 1;   %W_PY_N;
S_1(3,3) = 1;   %W_PZ_N;

% Minimize the trajectory
r_1 (:,:) = next_points(:,:);

ocp.minimizeLSQ( S_1, h_1, r_1 );

% Function h;
h = [ax_, ay_, az_];

% Weights
S = eye(3, 3);
S(1,1) = 1; %W_AX;
S(2,2) = 1; %W_AY;
S(3,3) = 1; %W_AZ;

% At the final point, use the theoretical acceleration to reach that point
r = [0, 0, 0];

ocp.minimizeLSQ( S, h, r );

% Set up the optimization algorithm
algo = acado.OptimizationAlgorithm(ocp);

% Set some parameters for the algorithm (can be commented)
algo.set('KKT_TOLERANCE', 1e-4 );
algo.set('INTEGRATOR_TOLERANCE', 1e-1 );
algo.set('RELAXATION_PARAMETER', 3 );

% Always end with "END_ACADO".
END_ACADO;

% -------------------------- END ACADO'S CODE -------------------------- %

% Now, let's run the test!
out = one_drone_simple_trajectory_RUN();

end