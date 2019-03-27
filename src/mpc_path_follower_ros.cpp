/*********************************************************************
 * @file:   MpcPathFollowerRos.cpp
 * @author: Lianchuan Zhang
 * @date:   2019.01.16
 * @version:1.0.1
 * @brief:  interface between mpc and ros navigation
 *********************************************************************/

#include <mpc_path_follower/mpc_path_follower_ros.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(mpc_path_follower::MpcPathFollowerRos, nav_core::BaseLocalPlanner)


namespace mpc_path_follower {
    MpcPathFollowerRos::MpcPathFollowerRos():initialized_(false),
        odom_helper_("odom"), setup_(false), debug_(true){

    }

    void MpcPathFollowerRos::initialize(std::string name,
                                           tf::TransformListener *tf,
                                           costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!isInitialized()) {

            ros::NodeHandle private_nh("~/" + name);
            ros::NodeHandle nh;
            nh.param<bool>("debug", debug_, true);
            nh.param<float>("path_length", pathLength_, 8.0); // unit: m
            g_plan_sub = nh.subscribe("global_plan", 1, &MpcPathFollowerRos::global_path_CB, this);
            //g_plan_sub = nh.subscribe("/control/speed", 1, &MpcPathFollowerRos::global_path_CB, this);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
            _pub_odompath  = nh.advertise<nav_msgs::Path>("/mpc_reference1", 1); // reference path for MPC
            _pub_mpctraj   = nh.advertise<nav_msgs::Path>("/mpc_trajectory1", 1);// MPC trajectory output
            _pub_vehicle_reftraj   = nh.advertise<nav_msgs::Path>("/vehicle_reftrajectory1", 1);// MPC trajectory output
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_);
            DT = 0.2;
            // make sure to update the costmap we'll use for this cycle
            costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

            planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

            if( private_nh.getParam( "odom_topic", odom_topic_ ))
            {
                odom_helper_.setOdomTopic( odom_topic_ );
            }

            initialized_ = true;
        }
        else{
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }

    bool MpcPathFollowerRos::computeVelocityCommands(geometry_msgs::Twist &cmd_vel){

        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
            ROS_ERROR("MPC Could not get local plan");
            return false;
        }

        //if the global plan passed in is empty... we won't do anything
        if(transformed_plan.empty()) {
            ROS_WARN_NAMED("mpc_local_planner", "Received an empty transformed plan.");
            return false;
        }
        ROS_FATAL_NAMED("mpc_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size()) ;

        if (isGoalReached())
        {
            //publish an empty plan because we've reached our goal position
            publishZeroVelocity();
        }
        else
        {
            bool isOk = mpcComputeVelocityCommands(transformed_plan, cmd_vel);
            if (isOk)
            {
                //ROS_ERROR("isOk is %d", isOk);
                //publishGlobalPlan(transformed_plan);
            }
            else
            {
                ROS_WARN_NAMED("mpc_local_planner", "mpc planner failed to produce path.");
                std::vector<geometry_msgs::PoseStamped> empty_plan;
                //publishGlobalPlan(empty_plan);
            }
            return isOk;
        }
    }

    bool MpcPathFollowerRos::mpcComputeVelocityCommands(std::vector<geometry_msgs::PoseStamped> &path, geometry_msgs::Twist &cmd_vel){

        tf::Stamped<tf::Pose> robot_vel;
        odom_helper_.getRobotVel(robot_vel);
        Eigen::Vector3f vel(robot_vel.getOrigin().getX(),
                            robot_vel.getOrigin().getY(), tf::getYaw(robot_vel.getRotation()));

        // Display the MPC referenced trajectory
        nav_msgs::Path _mpc_ref_traj;
        _mpc_ref_traj.header.frame_id = "odom"; // points in car coordinate
        _mpc_ref_traj.header.stamp = ros::Time::now();
        for(int i = 0; i < path.size(); i++)
        {
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = _mpc_ref_traj.header;
            tempPose.pose = path.at(i).pose;
            _mpc_ref_traj.poses.push_back(tempPose);
        }
        _pub_odompath.publish(_mpc_ref_traj);

        nav_msgs::Odometry odom;
        odom_helper_.getOdom(odom);
        double px = odom.pose.pose.position.x; //pose: odom frame
        double py = odom.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(odom.pose.pose, pose);
        double psi = tf::getYaw(pose.getRotation());
        // Waypoints related parameters
        double cospsi = cos(psi);
        double sinpsi = sin(psi);
        // Convert to the vehicle coordinate system
        std::vector<double> x_veh;
        std::vector<double> y_veh;
        x_veh.clear();
        y_veh.clear();

        nav_msgs::Path _vehicle_ref_traj;
        _vehicle_ref_traj.header.frame_id = "base_link"; // points in car coordinate
        _vehicle_ref_traj.header.stamp = ros::Time::now();
        for(int i = 0; i < path.size(); i++)
        {
            double dx = path.at(i).pose.position.x - px;
            double dy = path.at(i).pose.position.y - py;
            x_veh.push_back( dx * cospsi + dy * sinpsi);
            y_veh.push_back( dy * cospsi - dx * sinpsi);
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = _mpc_ref_traj.header;
            tempPose.pose.position.x = dx * cospsi + dy * sinpsi;
            tempPose.pose.position.y = dy * cospsi - dx * sinpsi;
            _vehicle_ref_traj.poses.push_back(tempPose);
        }
        _pub_vehicle_reftraj.publish(_vehicle_ref_traj);
        std::vector<double> waypoints_x;
        std::vector<double> waypoints_y;
        waypoints_x.clear();
        waypoints_y.clear();
        int i = 0;
        while (i < x_veh.size()){
            waypoints_x.push_back(x_veh.at(i));
            waypoints_y.push_back(y_veh.at(i));
            i = i + 10;
        }
        double* ptrx = &waypoints_x[0];
        double* ptry = &waypoints_y[0];
        Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
        Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);
        // calculate cte and epsi
        auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);        
        /* The cross track error is calculated by evaluating at polynomial at x, f(x)
        and subtracting y.
        double cte = polyeval(coeffs, x) - y;
        Due to the sign starting at 0, the orientation error is -f'(x).
        derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
        double epsi = psi - atan(coeffs[1]);*/
        double cte = polyeval(coeffs, 0);
        double epsi = atan(coeffs[1]);
        if (debug_){
            std::cout<<"psi is"<<std::endl;
            std::cout<<"path size is"<<path.size()<<std::endl;
            std::cout<<" x_veh size is "<<x_veh.size()<<std::endl;
            std::cout<<"waypoints x size is"<<waypoints_x.size()<<std::endl;
            std::cout<<"coeffs is "<<coeffs<<std::endl;
            std::cout<<"cte is"<<cte<<std::endl;
            std::cout<<"epsi is"<<epsi<<std::endl;
        }
        Eigen::VectorXd state(6);
        double dt = 0.1;

        state << 0, 0, 0, vel[0], cte, epsi;
        std::vector<double> vars;
        vars.clear();
        //mpc_solver.initialize();
        vars = mpc_solver.solve(state, coeffs);
        if (vars.size() < 2){
            return false;
        }
        std::vector<double> mpc_x_vals;
        std::vector<double> mpc_y_vals;
        mpc_x_vals.clear();
        mpc_y_vals.clear();
        std::cout<<"vars size is"<<vars.size()<<std::endl;
        for (int i = 2; i < vars.size(); i ++) {
          if (i%2 == 0) {
            mpc_x_vals.push_back(vars[i]);
            //std::cout<<"mpc x vals is "<<vars[i]<<std::endl;
          }
          else {
            mpc_y_vals.push_back(vars[i]);
          }
        }
        //std::cout<<"****************************************"<<std::endl;
        // Display the MPC predicted trajectory
        nav_msgs::Path _mpc_traj;
        //_mpc_traj.poses.clear();
        _mpc_traj.header.frame_id = "base_link"; // points in car coordinate
        _mpc_traj.header.stamp = ros::Time::now();
        for(int i = 2; i < mpc_x_vals.size() - 3; i++)
        {
            geometry_msgs::PoseStamped tempPose;
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = mpc_x_vals[i];
            tempPose.pose.position.y = mpc_y_vals[i];
            tempPose.pose.orientation.w = 1.0;
            _mpc_traj.poses.push_back(tempPose);
        }
        // publish the mpc trajectory

        _pub_mpctraj.publish(_mpc_traj);
        ROS_INFO("I am publishing mcp trak");
        double steer_value = 0.0, throttle_value = 0.0;
        steer_value = vars[0];
        throttle_value = vars[1];
        ROS_INFO("Steer value and throttle value is, %lf , %lf", steer_value, throttle_value);
        cmd_vel.linear.x = vel[0] + vars[1] * DT;
        cmd_vel.linear.x = std::max(0.2, cmd_vel.linear.x);
        double radius = 0.0;
        if (fabs(tan(steer_value)) <= 1e2){
            radius = 1e5;
        } else {
            radius = 0.5 / tan(steer_value);}
        cmd_vel.angular.z = std::max(-0.2, std::min(0.2, (cmd_vel.linear.x / radius)));
        //ROS_INFO("v value and z value is, %lf , %lf", cmd_vel.linear.x, cmd_vel.angular.z);
        //cmd_vel.linear.x = 0.0;
        return true;
    }

    bool MpcPathFollowerRos::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if(!initialized_){
            ROS_ERROR("Planner utils have not been initialized, please call initialize() first");
            return false;
        }
        //when we get a new plan, we also want to clear any latch we may have on goal tolerances
        latchedStopRotateController_.resetLatching();
        //oscillation_costs_.resetOscillationFlags();
        return planner_util_.setPlan(orig_global_plan);
    }

    void MpcPathFollowerRos::global_path_CB(const nav_msgs::Path& path){
        /*if(path.poses.empty())
            return;
        std::vector<geometry_msgs::PoseStamped> original_plan;
        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for(unsigned int i=0; i < path.poses.size(); i++){
            original_plan[i] = path.poses[i];
        }
        setPlan(original_plan);*/
    }

    bool MpcPathFollowerRos::isGoalReached(){
        if (! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }

        if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
            ROS_INFO("Goal reached");
            return true;
        } else {
            return false;
        }
    }

    double MpcPathFollowerRos::polyeval(Eigen::VectorXd coeffs, double x){
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

    Eigen::VectorXd MpcPathFollowerRos::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                                                   int order){
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++) {
            A(i, 0) = 1.0;
        }

        for (int j = 0; j < xvals.size(); j++) {
            for (int i = 0; i < order; i++) {
                A(j, i + 1) = A(j, i) * xvals(j);
            }
        }
        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    void MpcPathFollowerRos::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, l_plan_pub_);
    }

    void MpcPathFollowerRos::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, g_plan_pub_);
    }
};

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "mpc_path_follower_node");
//}
