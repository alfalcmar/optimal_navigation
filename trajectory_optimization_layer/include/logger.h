#ifndef LOGGER_H
#define LOGGER_H
#include <ros/ros.h>
#include<fstream>
#include <nav_msgs/Odometry.h>
#include <UAVState.h>
#include <ctime>
#include <string>
#include <ros/package.h>
#include <iostream>
#include <nav_msgs/Path.h>
class backendSolver; // forward declaration

namespace SolverUtils{
/**
 * This class log and provide rviz topics for visualization. This class use the pointer to a backendSolver class to log and visualize its data.
 */

class Logger{

private:
    std::ofstream file_;
    backendSolver* class_to_log_ptr_;
    ros::Publisher                 path_rviz_pub;          /**< Publisher for visualizing the generated trajectory on RVIZ */
    ros::Publisher                 target_path_rviz_pub;   /**< Publisher for visualizing the target trajectory on RVIZ */
    ros::Publisher                 path_no_fly_zone;       /**< Publisher for visualizing the no-fly zone RVIZ */

public:
    Logger(backendSolver* class_to_log_ptr_, ros::NodeHandle pnh);

    ~Logger();

    void logging();

    void loggingCalculatedTrajectory(const int solver_success);

    /*! \brief Publish a rectangle that represents a no fly zone in order to visualize on rviz
    *   \param point_1 2D vertice
    *   \param point_2 2D vertice
    **/
    void publishNoFlyZone(double point_1[2], double point_2[2], double point_3[2], double point_4[2]);
    /*! \brief Publish solved path to visualize on RVIZ
    **/
    void publishPath();
    /*! \brief Utility function to create a msg that allows us to visualize the target path prediction on RVIZ
    *   \return nav_msg to visualize
    **/
    nav_msgs::Path targetPathVisualization();
};

}

#endif