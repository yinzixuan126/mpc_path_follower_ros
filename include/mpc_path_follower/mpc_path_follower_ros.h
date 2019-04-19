/*********************************************************************
 * @file:   mpc_path_follower_ros.h
 * @author: Lianchuan Zhang
 * @date:   2019.01.16
 * @version:1.0.1
 * @brief:  interface between mpc and ros navigation
 *********************************************************************/
#ifndef _MPC_PATH_FOLLOWER_ROS_H
#define _MPC_PATH_FOLLOWER_ROS_H
#include <vector>
#include <math.h>
#include <mpc_path_follower/mpc_path_follower.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <navfn/navfn.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
namespace mpc_path_follower {
    class MpcPathFollowerRos : public nav_core::BaseLocalPlanner{
    public:
        /**
         * @brief  Constructor for mpc path follower wrapper
         */
        MpcPathFollowerRos();
        /**
         * @brief  Destructor for the wrapper
         */
        ~MpcPathFollowerRos()= default;
        /**
         * @brief  Constructs the ros wrapper
         * @param name The name to give this instance of the trajectory planner
         * @param tf A pointer to a transform listener
         * @param costmap The cost map to use for assigning costs to trajectories
         */
        void initialize(std::string name, tf::TransformListener* tf,
                        costmap_2d::Costmap2DROS* costmap_ros);
        /**
         * @brief  Given the current position, orientation, and velocity of the robot,
         * compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid trajectory was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief  Set the plan that the controller is following
         * @param orig_global_plan The plan to pass to the controller
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

        /**
         * @brief  Check if the goal pose has been achieved
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        bool isInitialized() {
            return initialized_;
        }

    private:

        inline void publishZeroVelocity(){
          geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
          vel_pub_.publish(cmd_vel);
        }
        /**
         * @brief evaluate a polynominal
         * @param coefficients and input
         * @return output of the polynominal
         */
        int ClosestWaypoint(double x, double y, nav_msgs::Path global_path)
        {}
        /**
         * @brief evaluate a polynominal
         * @param coefficients and input
         * @return output of the polynominal
         */
        double polyeval(Eigen::VectorXd coeffs, double x);

        /**
         * @brief fit a polynominal
         * @param vector x, vector y and order
         * @return output coefficients
         */
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                                int order);

        bool mpcComputeVelocityCommands(std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::Twist& cmd_vel );

        void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        void global_path_CB(const nav_msgs::Path& path);

        tf::TransformListener* tf_; ///< @brief Used for transforming point clouds
        tf::TransformListener _tf_listener;

        // for visualisation, publishers of global and local plan
        ros::Publisher g_plan_pub_, l_plan_pub_, vel_pub_;
        ros::Subscriber g_plan_sub, odom_sub;

        ros::Publisher _pub_odompath, _pub_mpctraj, _pub_vehicle_reftraj;

        costmap_2d::Costmap2DROS* costmap_ros_;

        bool setup_;

        tf::Stamped<tf::Pose> current_pose_;

        bool initialized_;
        bool debug_;

        base_local_planner::OdometryHelperRos odom_helper_;

        base_local_planner::LatchedStopRotateController latchedStopRotateController_;

        std::string odom_topic_;

        std::vector<geometry_msgs::PoseStamped> global_plan_;

        base_local_planner::LocalPlannerUtil planner_util_;

        Eigen::Vector3f vel;
        float  DT;
        float pathLength_;
        MPC_Path_Follower mpc_solver;
        bool _is_close_enough;

        std::vector<geometry_msgs::PoseStamped> temp_original_plan, temp_transformed_plan, final_transfromed_plan;

    };
};
#endif
