#include<backendSolver.h>
int main(int _argc, char **_argv)
{

    // ros node initialization
    ros::init(_argc, _argv,"solver");
    ros::NodeHandle pnh = ros::NodeHandle("~");
    ros::NodeHandle nh;
    backendSolver Solver;
    // TODO include or not calls to the uav from this node
    //go_to_waypoint_client = pnh.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");
    // take_off_srv = pnh.serviceClient<uav_abstraction_layer::TakeOff>("ual/take_off");
    // ros::Subscriber target_pose_sub = pnh.subscribe<nav_msgs::Odometry>(target_topic, 1, targetPoseCallback);
    //set_velocity_pub = pnh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity",1);


    // init solver
    if(!Solver.init(pnh, nh)){ // if the solver is correctly initialized
        // main loop
        while(ros::ok()){
            if(Solver.activated){
                // solver function
                x.clear();
                y.clear();
                z.clear();
                vx.clear();
                vy.clear();
                vz.clear();
                // call the solver function
                Solver.solver_success = solverFunction(x,y,z,vx,vy,vz, Solver.desired_pose, Solver.desired_vel, Solver.obst,Solver.target_vel);
                // TODO: why the definition of theses function are not here? This node should contain
                // every function that can be used with various solvers
                if(Solver.solver_success==1){
                    std::vector<double> yaw = Solver.predictingYaw(x,y,z,target_trajectory);
                    std::vector<double> pitch = Solver.predictingPitch(x,y,z,target_trajectory);
                    Solver.publishTrajectory(x,y,z,vx,vy,vz,yaw,pitch);
                }
                Solver.logToCsv(x,y,z,vx,vy,vz);
                Solver.target_path_rviz_pub.publish(Solver.targetPathVisualization()); 
                Solver.publishDesiredPoint(Solver.desired_pose[0], Solver.desired_pose[1], Solver.desired_pose[2]);
                Solver.publishPath(x,y,z);  
            }
            ros::spinOnce();
            sleep(2);
        }
    }else{
        ros::shutdown();
    }
}