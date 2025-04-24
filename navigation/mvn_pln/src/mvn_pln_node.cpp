#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include "actionlib_msgs/GoalStatus.h"
#include "tf/transform_listener.h"

#include "actionlib/client/simple_action_client.h"
#include "motion_synth/Joints.h"
#include "motion_synth/StartAndEndJoints.h"
#include "motion_synth/MotionSynthesisAction.h"

#define RATE 10

#define SM_INIT 0
#define SM_WAITING_FOR_TASK 1
#define SM_CALCULATE_PATH 2
#define SM_CHECK_IF_INSIDE_OBSTACLES 19
#define SM_WAITING_FOR_MOVE_BACKWARDS 18
#define SM_CHECK_IF_OBSTACLES 21
#define SM_WAIT_FOR_NO_OBSTACLES 22
#define SM_ENABLE_OBS_DETECT 23
#define SM_WAIT_FOR_OBS_DETECT 24
#define SM_START_MOVE_PATH 3
#define SM_WAIT_FOR_MOVE_FINISHED 4
#define SM_COLLISION_DETECTED 5
#define SM_STOP_RECEIVED 51
#define SM_CORRECT_FINAL_ANGLE 6
#define SM_WAIT_FOR_ANGLE_CORRECTED 7
#define SM_FINAL    17

bool stop = false;
bool collision_risk = false;
bool  patience = true;
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;

bool nav_goal_received = false;
geometry_msgs::Pose global_goal;
std::string base_link_name = "base_footprint";

bool arm_goal_received = false;
bool arm_goal_reached = false;
bool has_arm_start_pose = false;
bool has_arm_end_pose = false;
motion_synth::StartAndEndJoints target_arm_pose;


void callback_general_stop(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "MvnPln.->General Stop signal received" << std::endl;
    stop = true;
}

void callback_navigation_stop(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "MvnPln.->Navigation Stop signal received" << std::endl;
    stop = true;
}

void callback_simple_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    global_goal = msg->pose;
    nav_goal_received = true;
}

void callback_arm_joints(const motion_synth::StartAndEndJoints::ConstPtr& msg)
{
    target_arm_pose = *msg;
    arm_goal_received = true;
}

void callback_collision_risk(const std_msgs::Bool::ConstPtr& msg){
    collision_risk = msg->data;
}

void callback_simple_move_goal_status(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    simple_move_goal_status = *msg;
    std::stringstream ss;
    ss << msg->goal_id.id;
    ss >> simple_move_status_id;
}

void callback_set_patience(const std_msgs::Bool::ConstPtr& msg)
{
    std::cout << "MvnPln.->Set patience: " << (msg->data ? "True" : "False") << std::endl;
    patience = msg->data;
}

bool plan_path_from_augmented_map(float robot_x, float robot_y, float goal_x, float goal_y, ros::ServiceClient& clt, nav_msgs::Path& path)
{
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = robot_x;
    srv.request.start.pose.position.y = robot_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    if(!clt.call(srv))
        return false;
    path = srv.response.plan;
    return true;
}

void get_robot_position(tf::TransformListener& listener, float& robot_x, float& robot_y, float& robot_a){
    tf::StampedTransform tf;
    listener.lookupTransform("map", base_link_name, ros::Time(0), tf);
    robot_x = tf.getOrigin().x();
    robot_y = tf.getOrigin().y();

    tf::Quaternion q = tf.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    robot_a = yaw;

    //q = tf.getRotation();
    //robot_a = atan2((float)q.z(), (float)q.w()) * 2;
}

int publish_status(int status, int id, std::string text, ros::Publisher& pub)
{
    actionlib_msgs::GoalStatus msg;
    std::stringstream ss;
    ss << id;
    msg.status = status;
    msg.text   = text;
    ss >> msg.goal_id.id;
    msg.goal_id.stamp = ros::Time::now();
    pub.publish(msg);
    return status;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mvn_pln");
    ROS_INFO("INITIALIZING MOVING PLANNER NODE BY MARCO NEGRETE..." );

    ros::NodeHandle n;
    tf::TransformListener listener;
    
    float proximity_criterion = 2.0;
    //float move_error_threshold = 0.03;

    if(ros::param::has("~patience"))
        ros::param::get("~patience", patience);
    if(ros::param::has("~proximity_criterion"))
        ros::param::get("~proximity_criterion", proximity_criterion);
    if(ros::param::has("/base_link_name"))
        ros::param::get("/base_link_name", base_link_name);
    //if(ros::param::has("~move_error_threshold"))
    //    ros::param::get("~move_error_threshold", move_error_threshold);

    std::cout << "MvnPln.->Patience: " << (patience?"True":"False") << "  Proximity criterion: " << proximity_criterion;
    std::cout << "  base link name: " << base_link_name << std::endl;
    //std::cout << "  move error threshold: " << move_error_threshold << std::endl;
    std::cout << "MvnPln.->Waiting for localization transform..." << std::endl;
    listener.waitForTransform("map", base_link_name, ros::Time(0), ros::Duration(10.0));
    std::cout << "MvnPln.->Localization transform is ready ..." << std::endl;
    std::cout << "MvnPln.->Waiting for path planner to be ready..." << std::endl;
    ros::service::waitForService("/path_planner/plan_path_with_static"   , ros::Duration(10));
    ros::service::waitForService("/path_planner/plan_path_with_augmented", ros::Duration(10));
    std::cout << "MvnPln.->Path planner is ready." << std::endl;
    std::cout << "mvnPln.->Waiting for map augmenter to be ready..." << std::endl;
    ros::service::waitForService("/map_augmenter/get_augmented_map"     , ros::Duration(10));
    ros::service::waitForService("/map_augmenter/get_augmented_cost_map", ros::Duration(10));
    ros::service::waitForService("/map_augmenter/are_there_obstacles"   , ros::Duration(10));
    std::cout << "MvnPln.->Map Augmenter is ready..." << std::endl;
    
    ros::Subscriber sub_generalStop        = n.subscribe("/stop", 10, callback_general_stop);
    ros::Subscriber sub_navCtrlStop        = n.subscribe("/navigation/stop", 10, callback_navigation_stop);               
    ros::Subscriber sub_simple_goal        = n.subscribe("/nav_control/goal", 10, callback_simple_goal);
    ros::Subscriber sub_arm_goal     = n.subscribe<motion_synth::StartAndEndJoints>("/pumas_motion_synth/joint_pose", 10, callback_arm_joints);
    ros::Subscriber sub_collision_risk     = n.subscribe("/navigation/obs_detector/collision_risk", 10, callback_collision_risk);
    ros::Subscriber sub_move_goal_status   = n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    ros::Subscriber sub_set_patience       = n.subscribe("/navigation/set_patience", 10, callback_set_patience);
    ros::Publisher pub_obs_detector_enable = n.advertise<std_msgs::Bool>("/navigation/obs_detector/enable", 1);
    ros::Publisher pub_goal_path           = n.advertise<nav_msgs::Path>("/simple_move/goal_path", 1);
    ros::Publisher pub_goal_dist_angle     = n.advertise<std_msgs::Float32MultiArray>("/simple_move/goal_dist_angle", 1);
    ros::Publisher pub_status              = n.advertise<actionlib_msgs::GoalStatus>("/navigation/status", 10);
    ros::Publisher pub_simple_move_stop    = n.advertise<std_msgs::Empty>("/simple_move/stop", 1);

    ros::Publisher pub_tmp_head_pose_cancel    = n.advertise<std_msgs::Empty>("/navigation/tmp_head_pose_cancel", 1);

    ros::ServiceClient clt_plan_path       = n.serviceClient<nav_msgs::GetPlan>("/path_planner/plan_path_with_augmented");
    ros::ServiceClient clt_are_there_obs   = n.serviceClient<std_srvs::Trigger>("/map_augmenter/are_there_obstacles");
    ros::ServiceClient clt_is_in_obstacles = n.serviceClient<std_srvs::Trigger>("/map_augmenter/is_inside_obstacles");

    // for motion synth_action_client add by r.k 2025/04/10
    typedef actionlib::SimpleActionClient<motion_synth::MotionSynthesisAction> MotionSynthActionClient;
    MotionSynthActionClient* ms_ac;
    ms_ac = new MotionSynthActionClient("/motion_synth", true);
    ROS_INFO("MvnPln.->Waiting for motion_synth action server");
    ms_ac->waitForServer(ros::Duration(5.0));
    ROS_INFO("MvnPln.->Connected to motion_synth action server");
    //

    ros::Rate loop(RATE);
    ros::Rate slow_loop(1);

    float robot_x = 0;
    float robot_y = 0;
    float robot_a = 0;
    float error = 0;

    int  state = SM_INIT;
    int  simple_move_sequencer = -1;
    int  goal_id = -1;
    int  current_status = 0;
    bool near_goal_sent = false;
    int near_goal_counter = 0;
    std_msgs::Bool msg_bool;
    std_srvs::Trigger srv_check_obstacles;
    nav_msgs::Path path;
    std_msgs::Float32MultiArray msg_goal_dist_angle;
    msg_goal_dist_angle.data.resize(2);


    while(ros::ok())
    {
        if(stop)
        {
            stop = false;
            state = SM_INIT;
            if(current_status == actionlib_msgs::GoalStatus::ACTIVE)
                current_status=publish_status(actionlib_msgs::GoalStatus::ABORTED,goal_id, "Stop signal received. Task cancelled", pub_status);
        }

        if(nav_goal_received)
            state = SM_WAITING_FOR_TASK;

        switch(state)
        {
            case SM_INIT:
                ROS_WARN("Pumas Navigation. -> Ready!!!!!!!!! ");
                ROS_WARN("MvnPln.-> MVN PLN READY. Waiting For New Goal.");
                state = SM_WAITING_FOR_TASK;
                break;

            case SM_WAITING_FOR_TASK:
                
                //motion synth add by r.k 2025/04/10
                if (arm_goal_received)
                {
                    motion_synth::MotionSynthesisGoal motion_synth_goal;
                    motion_synth_goal.goal_location.x = global_goal.position.x;
                    motion_synth_goal.goal_location.y = global_goal.position.y;
                    motion_synth_goal.goal_location.theta = atan2(global_goal.orientation.z, global_goal.orientation.w) * 2;
                    if (target_arm_pose.has_arm_start_pose == true)
                    {
                        motion_synth_goal.apply_start_pose = true;
                        motion_synth_goal.start_pose = target_arm_pose.start_pose;
                    } 
                    if (target_arm_pose.has_arm_end_pose == true)
                    {
                        motion_synth_goal.apply_goal_pose = true;
                        motion_synth_goal.goal_pose = target_arm_pose.end_pose;
                    }
                    arm_goal_received = false;
                    ms_ac->sendGoal(motion_synth_goal);
                    ROS_INFO("MvnPln.->Arm motion goal sent");
                }

                if(nav_goal_received)
                {
                    nav_goal_received = false;
                    state = SM_CALCULATE_PATH;
                    if(current_status == actionlib_msgs::GoalStatus::ACTIVE)
                        current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id, "Cancelling current movement.", pub_status);
                    goal_id++;
                    ROS_INFO("MvnPln.->New goal received. Current task goal_id: ");
                    current_status = publish_status(actionlib_msgs::GoalStatus::ACTIVE, goal_id, "Starting new movement task", pub_status);
                    near_goal_sent = false;
                }
                break;
                
            //case SM_CALCULATE_PATH:
            //    get_robot_position(listener, robot_x, robot_y, robot_a);
            //    if(!plan_path_from_augmented_map(robot_x, robot_y, global_goal.position.x, global_goal.position.y, clt_plan_path, path))
            //    {
            //        std::cout<<"MvnPln.->Cannot calc path to "<< global_goal.position.x <<" "<<global_goal.position.y << std::endl;
            //        pub_simple_move_stop.publish(std_msgs::Empty());

            //        // if is inside STATIC obstacle, go backwards. add by r.k 2025/04/11 //TODO
            //        if (clt_is_in_obstacles.call(srv_check_obstacles) && srv_check_obstacles.response.success)
            //        {
            //            ROS_ERROR("MvnPln.->Robot is inside an obstacle. Will attempt recovery with going backwards.");
            //            state = SM_CHECK_IF_INSIDE_OBSTACLES;
            //        }
            //        else
            //        {
            //            //original
            //            if(!patience)
            //                state = SM_CHECK_IF_INSIDE_OBSTACLES;
            //            else
            //                state = SM_CHECK_IF_OBSTACLES;
            //        }
            //    }
            //    else
            //    {
            //        state = SM_ENABLE_OBS_DETECT;
            //    }
            //    break;

            case SM_CALCULATE_PATH:
                get_robot_position(listener, robot_x, robot_y, robot_a);
                if(!plan_path_from_augmented_map(robot_x, robot_y, global_goal.position.x, global_goal.position.y, clt_plan_path, path))
                {
                    std::cout<< "\033[31m" << "MvnPln.->Cannot calc path to "<< global_goal.position.x <<" "<<global_goal.position.y << "\033[0m]" << std::endl;

                    pub_simple_move_stop.publish(std_msgs::Empty());
                    if(!patience)
                        state = SM_CHECK_IF_INSIDE_OBSTACLES;
                    else
                        state = SM_CHECK_IF_OBSTACLES;
                }
                else
                    state = SM_ENABLE_OBS_DETECT;
                break;

            case SM_CHECK_IF_INSIDE_OBSTACLES:
                ROS_INFO("MvnPln.->Checking if robot is inside an obstacle...");
                clt_is_in_obstacles.call(srv_check_obstacles);
                if(srv_check_obstacles.response.success)
                {
                    ROS_INFO("MvnPln.->Robot is inside an obstacle. Moving backwards...");
                    msg_goal_dist_angle.data[0] = -0.25;
                    msg_goal_dist_angle.data[1] = 0;
                    pub_goal_dist_angle.publish(msg_goal_dist_angle);

                    state = SM_WAITING_FOR_MOVE_BACKWARDS;
                }
                else
                {
                    current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id, "Cannot calc path from start to goal", pub_status);
                    state = SM_INIT;
                }
                break;

            case SM_WAITING_FOR_MOVE_BACKWARDS:
                if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1)
                {
                    simple_move_goal_status.status = 0;
                    ROS_INFO("MvnPln.->Moved backwards succesfully.");
                }
                else if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::ABORTED)
                {
                    simple_move_goal_status.status = 0;
                    ROS_ERROR("MvnPln.->Simple move reported move aborted. ");
                }
                state = SM_CALCULATE_PATH;
                break;
                
            case SM_CHECK_IF_OBSTACLES:
                if(!clt_are_there_obs.call(srv_check_obstacles) || !srv_check_obstacles.response.success)
                {
                    ROS_ERROR("MvnPln.->There are no temporal obstacles. Announcing failure.");
                    current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id,
                                                    "Cannot calculate path from start to goal point", pub_status);
                    state = SM_INIT;
                }
                else
                {
                    ROS_WARN("MvnPln.->Temporal obstacles detected. Waiting for them to move.");
                    current_status = publish_status(actionlib_msgs::GoalStatus::ACTIVE, goal_id,
                                                    "Waiting for temporal obstacles to move", pub_status);
                    state = SM_WAIT_FOR_NO_OBSTACLES;
                }
                break;

            case SM_WAIT_FOR_NO_OBSTACLES:
                if(!clt_are_there_obs.call(srv_check_obstacles))
                {
                    ROS_ERROR("MvnPln.->Cannot call service for checking temporal obstacles. Announcing failure.");
                    current_status = publish_status(actionlib_msgs::GoalStatus::ABORTED, goal_id,
                                                    "Cannot calculate path from start to goal point", pub_status);
                    state = SM_INIT;
                }
                else if(!srv_check_obstacles.response.success)
                {
                    ROS_INFO("MvnPln.->Temporal obstacles removed. ");
                    state = SM_CALCULATE_PATH;
                }
                else
                    slow_loop.sleep();
                break;
                
            case SM_ENABLE_OBS_DETECT:
                msg_bool.data = true;
                pub_obs_detector_enable.publish(msg_bool);
                ROS_INFO("MvnPln.->Obstacle detector enable flag sent. Waiting for obs detector to be enabled..." );
                state = SM_WAIT_FOR_OBS_DETECT;
                break;
                
            case SM_WAIT_FOR_OBS_DETECT:
                ros::topic::waitForMessage<std_msgs::Bool>("/navigation/obs_detector/collision_risk", ros::Duration(100.0));
                ROS_INFO("MvnPln.->Obstacle detector is now available.");
                state = SM_START_MOVE_PATH;
                break;

            case SM_START_MOVE_PATH:
                ROS_INFO("MvnPln.->Starting path following");
                collision_risk = false;
                simple_move_sequencer++;
                path.header.seq = simple_move_sequencer;
                
                //add 2024/09/12
                if (path.poses.size() > 0){
                    pub_goal_path.publish(path);
                    simple_move_goal_status.status = 0;
                    state = SM_WAIT_FOR_MOVE_FINISHED;
                }
                else
                {
                    state = SM_CORRECT_FINAL_ANGLE;
                    //state = SM_FINAL;
                }

                //add for hsrc because sometimes occured self collision //add by r.k 2025/04/23
                pub_tmp_head_pose_cancel.publish(std_msgs::Empty());

                break;

            case SM_WAIT_FOR_MOVE_FINISHED:
                //motion synth add by r.k 2025/04/10
                if (ms_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO_THROTTLE(1.0, "MvnPln.->Arm motion finished successfully.");
                }
                //else
                //{
                //    ROS_INFO_THROTTLE(1.0, "MvnPln.->Arm motion not completed and not running (state: %s)", ms_ac->getState().toString().c_str());
                //}

                //navigation goal
                //if(error < proximity_criterion && !near_goal_sent)
                //if(error < proximity_criterion && !near_goal_sent && error > 0.03) //hsrb
                //if(error < proximity_criterion && !near_goal_sent && error > 0.001) //sim
                
                get_robot_position(listener, robot_x, robot_y, robot_a);
                error = sqrt(pow(global_goal.position.x - robot_x, 2) + pow(global_goal.position.y - robot_y, 2));
                ROS_INFO_THROTTLE(1.0, "MvnPln. -> will move error: %f", error);
                if (error < proximity_criterion && !near_goal_sent)
                {
                    near_goal_counter++;

                    near_goal_sent = true;
                    ROS_INFO("MvnPln.->Error less than proximity criterion. Sending near goal point status.");
                    publish_status(actionlib_msgs::GoalStatus::ACTIVE, goal_id, "Near goal point", pub_status);

                }
                if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == simple_move_sequencer)
                {
                    simple_move_goal_status.status = 0;
                    ROS_INFO("MvnPln.->Path followed succesfully. ");
                    msg_bool.data = false;
                    pub_obs_detector_enable.publish(msg_bool);
                    state = SM_CORRECT_FINAL_ANGLE;
                }
                else if(collision_risk)
                {
                    ROS_INFO("MvnPln.->COLLISION RISK DETECTED before goal is reached.");
                    collision_risk = false;
                    state = SM_CALCULATE_PATH;
                }
                else if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::ABORTED)
                {
                    simple_move_goal_status.status = 0;
                    ROS_ERROR("MvnPln.->Simple move reported path aborted. Trying again...");
                    
                    //motion synth
                    if (ms_ac->getState() == actionlib::SimpleClientGoalState::ACTIVE)
                    {
                        ms_ac->cancelGoal();
                        ROS_WARN("MvnPln->Canceling arm motion due to move abort.");
                    }
                    state = SM_CALCULATE_PATH;
                }
                break;

            case SM_CORRECT_FINAL_ANGLE:
                {
                    std::cout << "MvnPln.->Correcting final angle." << std::endl;
                    get_robot_position(listener, robot_x, robot_y, robot_a);
                    //error = atan2(global_goal.orientation.z, global_goal.orientation.w)*2 - robot_a;
                    
                    tf::Quaternion q(
                        global_goal.orientation.x,
                        global_goal.orientation.y,
                        global_goal.orientation.z,
                        global_goal.orientation.w
                    );
                    float goal_a = tf::getYaw(q);
                    error = goal_a - robot_a;

                    if(error  >  M_PI) error -= 2*M_PI;
                    if(error <= -M_PI) error += 2*M_PI;
                    msg_goal_dist_angle.data[0] = 0;
                    msg_goal_dist_angle.data[1] = error;
                    pub_goal_dist_angle.publish(msg_goal_dist_angle);
                    state = SM_WAIT_FOR_ANGLE_CORRECTED;
                    break;
                }

            case SM_WAIT_FOR_ANGLE_CORRECTED:
                if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1)
                {
                    simple_move_goal_status.status = 0;
                    ROS_INFO("MvnPln.->Final angle corrected succesfully.");
                    state = SM_FINAL ;
                }
                else if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::ABORTED)
                {
                    simple_move_goal_status.status = 0;
                    ROS_ERROR("MvnPln.->Simple move reported path aborted. Trying again...");
                    state = SM_CALCULATE_PATH;
                }
                break;
                
            case SM_FINAL:
                
                // motion_synth forcing cancel
                /*
                if (arm_goal_received &&
                    ms_ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED &&
                    ms_ac->getState() != actionlib::SimpleClientGoalState::ABORTED &&
                    ms_ac->getState() != actionlib::SimpleClientGoalState::PREEMPTED)
                {
                    ROS_ERROR("MvnPln.->Navigation Finished. But arm motion still running, forcing cancel.");
                    ms_ac->cancelGoal();
                }
                */

                /* if (target_arm_pose.has_arm_end_pose) */
                /* { */
                /*     if (ms_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) */
                /*     { */
                /*         ROS_INFO("MvnPln.->Arm motion completed successfully."); */
                /*     } */
                /*     else if(ms_ac->getState().isDone()) */
                /*     { */
                /*         ROS_WARN("MvnPln.->Arm motion completed with state: %s", ms_ac->getState().toString().c_str()); */
                /*     } */
                /*     else */
                /*     { */
                /*         ROS_INFO_THROTTLE(1.0, "MvnPln.->Waiting for arm motion to complete"); */
                /*         break; */
                /*     } */
                /* } */

                std::cout << "MvnPln.->TASK FINISHED." << std::endl;
                current_status = publish_status(actionlib_msgs::GoalStatus::SUCCEEDED, goal_id, "Global goal point reached", pub_status);
                state = SM_INIT;
                break;
    
            default:
                std::cout<<"MvnPln.->A VERY STUPID PERSON PROGRAMMED THIS SHIT. APOLOGIES FOR THE INCONVINIENCE. Please contact your dealer."<<std::endl;
        }

        ros::spinOnce();
        loop.sleep();

    }
}
