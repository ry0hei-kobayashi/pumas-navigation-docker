#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Trigger.h"
#include "PathPlanner.h"
#include "path_planner/GetPlanWithVia.h"

ros::ServiceClient cltGetStaticMap       ;
ros::ServiceClient cltGetStaticCostMap   ;
ros::ServiceClient cltGetAugmentedMap    ;
ros::ServiceClient cltGetAugmentedCostMap;

float smooth_alpha   = 0.1;
float smooth_beta    = 0.9;
bool diagonal_paths = false;
bool path_plan_success = false;

bool callback_path_plan_status(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    resp.success = path_plan_success;
    resp.message = path_plan_success ? "Path planning succeeded." : "Path planning failed.";
    return true;
}

bool callback_a_star_with_static_map(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
    std::cout << "PathPlanner.-> Received path planning request using static map..." << std::endl;
    nav_msgs::GetMap srvStaticMap;
    nav_msgs::GetMap srvCostMap;
    if(!cltGetStaticMap.call(srvStaticMap))
    {
        std::cout << "PathPlanner.-> Cannot get static map!!!!" << std::endl;
        return false;
    }
    if(!cltGetStaticCostMap.call(srvCostMap))
    {
        std::cout << "PathPlanner.-> Cannot get static cost map!!!!" << std::endl;
        return false;
    }
    bool success = PathPlanner::AStar(srvStaticMap.response.map, srvCostMap.response.map,
                                         req.start.pose, req.goal.pose, diagonal_paths, resp.plan);
    if(success){
        resp.plan = PathPlanner::SmoothPath(resp.plan, smooth_alpha, smooth_beta);
    	path_plan_success = true;
    }
    else
    {
        std::cout << "PathPlanner.-> Cannot calculte path from start to goal positions using static map..." << std::endl;
    	path_plan_success = false;
    }
    return success;
}

bool callback_a_star_with_augmented_map(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
    std::cout << "PathPlanner.-> Received path planning request usign augmented map..." << std::endl;
    nav_msgs::GetMap srvStaticMap;
    nav_msgs::GetMap srvCostMap;
    if(!cltGetAugmentedMap.call(srvStaticMap))
    {
        std::cout << "PathPlanner.-> Cannot get augmented map!!!!" << std::endl;
        return false;
    }
    if(!cltGetAugmentedCostMap.call(srvCostMap))
    {
        std::cout << "PathPlanner.-> Cannot get augmented cost map!!!!" << std::endl;
        return false;
    }
    bool success = PathPlanner::AStar(srvStaticMap.response.map, srvCostMap.response.map,
                                         req.start.pose, req.goal.pose, diagonal_paths, resp.plan);
    if(success)
    {
        resp.plan = PathPlanner::SmoothPath(resp.plan, smooth_alpha, smooth_beta);
    	path_plan_success = true;
    }
    else
    {
        std::cout << "PathPlanner.-> Cannot calculte path from start to goal positions using augmented map..." << std::endl;
    	path_plan_success = false;
    }
    return success;
}

//add by ry0hei k 2025/06/12
bool callback_astar_service(path_planner::GetPlanWithVia::Request& req, path_planner::GetPlanWithVia::Response& resp)
{
    nav_msgs::GetMap srvMap, srvCostMap;

    //use augmented maps!!!
    if (!cltGetAugmentedMap.call(srvMap)) return false;
    if (!cltGetAugmentedCostMap.call(srvCostMap)) return false;

    nav_msgs::Path raw_path;
    bool success = false;

    if (req.via_points.empty()) {
        success = PathPlanner::AStar(srvMap.response.map, srvCostMap.response.map,
                                     req.start.pose, req.goal.pose, diagonal_paths, raw_path);
    } else {
        std::vector<geometry_msgs::Pose> via_poses;
        for (auto& vp : req.via_points) via_poses.push_back(vp.pose);
        success = PathPlanner::AStarWithViaPoints(srvMap.response.map, srvCostMap.response.map,
                                                  req.start.pose, via_poses, req.goal.pose, diagonal_paths, raw_path);
    }

    if (success) {
        resp.plan = PathPlanner::SmoothPath(raw_path, smooth_alpha, smooth_beta);
        path_plan_success = true;
    } else {
        path_plan_success = false;
    }
    return success;
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING PATH CALCULATOR BY MARCO NEGRETE..." << std::endl;
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n("~");

    if(ros::param::has("~diagonal_paths"))
    	ros::param::get("~diagonal_paths", diagonal_paths);
    if(ros::param::has("~smooth_alpha"))
    	ros::param::get("~smooth_alpha", smooth_alpha);
    if(ros::param::has("~smooth_beta"))
    	ros::param::get("~smooth_beta", smooth_beta);
    
    ros::Rate loop(10);
    std::cout << "PathPlanner.->Calculate diagonal paths: " << (diagonal_paths? "True" : "False") << std::endl;
    std::cout << "PathPlanner.->Smooth alpha: " << smooth_alpha << "   smooth beta: " << smooth_beta << std::endl;
    std::cout << "PathPlanner.->Waiting for map services..." << std::endl;
    ros::service::waitForService("/map_augmenter/get_static_map"        , ros::Duration(1000.0));
    ros::service::waitForService("/map_augmenter/get_static_cost_map"   , ros::Duration(1000.0));
    ros::service::waitForService("/map_augmenter/get_augmented_map"     , ros::Duration(1000.0));
    ros::service::waitForService("/map_augmenter/get_augmented_cost_map", ros::Duration(1000.0));
    std::cout << "PathPlanner.->Augmented Map services are now available..." << std::endl;

    //The following service gets the static map plus the prohibition layer
    cltGetStaticMap        = n.serviceClient<nav_msgs::GetMap>("/map_augmenter/get_static_map"        ); 
    cltGetStaticCostMap    = n.serviceClient<nav_msgs::GetMap>("/map_augmenter/get_static_cost_map"   );
    cltGetAugmentedMap     = n.serviceClient<nav_msgs::GetMap>("/map_augmenter/get_augmented_map"     );
    cltGetAugmentedCostMap = n.serviceClient<nav_msgs::GetMap>("/map_augmenter/get_augmented_cost_map");

    ros::ServiceServer srvPathPlanStatus = n.advertiseService("/path_planner/path_plan_status", callback_path_plan_status);
    ros::ServiceServer srvGetPlanStatic   =n.advertiseService("/path_planner/plan_path_with_static"   , callback_a_star_with_static_map);
    ros::ServiceServer srvGetPlanAugmented=n.advertiseService("/path_planner/plan_path_with_augmented", callback_a_star_with_augmented_map);

    //add by ry0hei k 2025/06/12
    ros::ServiceServer srvGetPlanUnified = n.advertiseService("/path_planner/plan_path", callback_astar_service);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
