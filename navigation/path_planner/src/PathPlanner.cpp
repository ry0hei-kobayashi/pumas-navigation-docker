#include "PathPlanner.h"
#include <cmath>
#include <cstdlib>

//original Astar func
bool PathPlanner::AStar(nav_msgs::OccupancyGrid& map, nav_msgs::OccupancyGrid& cost_map,
                      geometry_msgs::Pose& start_pose, geometry_msgs::Pose& goal_pose, bool diagonal_paths, nav_msgs::Path& result_path)
{
    std::cout << "PathPlanner.-> Calculating by A* from " << start_pose.position.x << "  ";
    std::cout << start_pose.position.y << "  to " << goal_pose.position.x << "  " << goal_pose.position.y << std::endl;

    int idx_start_x;
    int idx_start_y;
    int idx_goal_x;
    int idx_goal_y;
    idx_start_y = (int)((start_pose.position.y - map.info.origin.position.y)/map.info.resolution);
    idx_start_x = (int)((start_pose.position.x - map.info.origin.position.x)/map.info.resolution);
    int idx_start = idx_start_y*map.info.width + idx_start_x;


    //double distance = 0.1;
    //double angle = 1.414213562;
    idx_goal_y  = (int)((goal_pose.position.y  - map.info.origin.position.y)/map.info.resolution);
    idx_goal_x  = (int)((goal_pose.position.x  - map.info.origin.position.x)/map.info.resolution);
    int _idx_goal_y = idx_goal_y;
    int _idx_goal_x = idx_goal_x;
    int idx_goal  = idx_goal_y *map.info.width + idx_goal_x;

    int count = 0;
    int loop_count = 0;
    double radius = 0.1;
    
    int MAX_GOAL_UPDATE = 16; //points on circle
    double angle_increment = 2 * M_PI / (MAX_GOAL_UPDATE - 1);
    while (map.data[idx_goal] != 0 or loop_count < 4)
    //if(map.data[idx_goal] != 0)
    {

        double angle = count * angle_increment;
        //ROS_ERROR("PathPlanner.->Goal point is inside non-free space!!!!");
        idx_goal_y  = _idx_goal_y + radius * cos(angle);
        idx_goal_x  = _idx_goal_x + radius * sin(angle);
        idx_goal  = idx_goal_y *map.info.width + idx_goal_x;
	
        count++;
        if (count=MAX_GOAL_UPDATE)
        {
            count = 0;
            radius += 0.1;
            loop_count++;
        }
    }

    if (map.data[idx_goal] != 0)
    {
        ROS_ERROR("PathPlanner.->Goal point is inside non-free space!!!!");
        return false;
    } 

    if(map.data[idx_start] != 0)
    {
        ROS_ERROR("PathPlanner.->Start point is inside non-free space!!!!");
        return false;
    }
    
    

    std::vector<Node> nodes;
    Node* current_node; 
    std::vector<int> node_neighbors;
    int steps = 0;
    nodes.resize(map.data.size());
    if(diagonal_paths)
        node_neighbors.resize(8);
    else
        node_neighbors.resize(4);
    std::priority_queue<Node*, std::vector<Node*>, CompareByFValue>   open_list;
    for(size_t i=0;  i < map.data.size(); i++)
	nodes[i].index = i;

    current_node = &nodes[idx_start];
    current_node->g_value      = 0;
    current_node->in_open_list = true;    
    open_list.push(current_node);
    
    while(!open_list.empty() && current_node->index != idx_goal)
    {
    
        current_node = open_list.top();  
        open_list.pop();                   
        current_node->in_closed_list = true;

        node_neighbors[0] = current_node->index + map.info.width;
        node_neighbors[1] = current_node->index + 1;
        node_neighbors[2] = current_node->index - map.info.width;
        node_neighbors[3] = current_node->index - 1;
        if(diagonal_paths)
        {
            node_neighbors[4] = current_node->index + map.info.width + 1;
            node_neighbors[5] = current_node->index + map.info.width - 1;
            node_neighbors[6] = current_node->index - map.info.width + 1;
            node_neighbors[7] = current_node->index - map.info.width - 1;
        }
       
        for(size_t i=0; i < node_neighbors.size(); i++)
        {
            if(map.data[node_neighbors[i]] != 0 || nodes[node_neighbors[i]].in_closed_list)
                continue;
       
            Node* neighbor = &nodes[node_neighbors[i]];
            float delta_g = i < 4 ? 1.0 : 1.414213562;
            float g_value = current_node->g_value + (i < 4 ? 1.0 : 1.414213562) + cost_map.data[node_neighbors[i]];
            float h_value;
            int   h_value_x = node_neighbors[i]%map.info.width - idx_goal_x;
            int   h_value_y = node_neighbors[i]/map.info.width - idx_goal_y;
            if(diagonal_paths)
                h_value = sqrt(h_value_x*h_value_x + h_value_y*h_value_y);
            else
                h_value = fabs(h_value_x) + fabs(h_value_y);
            
            if(g_value < neighbor->g_value)
            {
                neighbor->g_value = g_value;
                neighbor->f_value = g_value + h_value;
                neighbor->parent  = current_node;
            }

            if(!neighbor->in_open_list)
            {
                neighbor->in_open_list = true;
                open_list.push(neighbor);
            }
        }
        steps++;
    }
    std::cout << "PathPlanner.->A* Algorithm ended after " << steps << " steps" << std::endl;

    if(current_node->index != idx_goal)
	return false;
    
    result_path.header.frame_id = "map";
    result_path.poses.clear();
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    while(current_node->parent != NULL)
    {
	p.pose.position.x = current_node->index % map.info.width * map.info.resolution + map.info.origin.position.x;	
	p.pose.position.y = current_node->index / map.info.width * map.info.resolution + map.info.origin.position.y;
	result_path.poses.insert(result_path.poses.begin(), p);
	current_node = current_node->parent;
    }
    
    std::cout << "PathPlanner.->Resulting path by A* has " << result_path.poses.size() << " points." << std::endl;
    return true;
}

// via point bias 
void PathPlanner::addViaPointBias(nav_msgs::OccupancyGrid& cost_map,
                                   const geometry_msgs::Pose& via_pose,
                                   double radius, int cost_bias)
{
    int width = cost_map.info.width;
    int height = cost_map.info.height;
    double resolution = cost_map.info.resolution;
    double origin_x = cost_map.info.origin.position.x;
    double origin_y = cost_map.info.origin.position.y;

    int center_x = (int)((via_pose.position.x - origin_x) / resolution);
    int center_y = (int)((via_pose.position.y - origin_y) / resolution);
    int cell_radius = (int)(radius / resolution);

    for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
            int x = center_x + dx;
            int y = center_y + dy;
            if (x >= 0 && x < width && y >= 0 && y < height) {
                int idx = y * width + x;
                double dist = std::sqrt(dx * dx + dy * dy) * resolution;
                if (dist <= radius) {
                    cost_map.data[idx] = std::max(0, cost_map.data[idx] - cost_bias); 
                }
            }
        }
    }
}

// A* with via points function
bool PathPlanner::AStarWithViaPoints(nav_msgs::OccupancyGrid& map,
                                     nav_msgs::OccupancyGrid& cost_map,
                                     geometry_msgs::Pose& start_pose,
                                     std::vector<geometry_msgs::Pose>& via_poses,
                                     geometry_msgs::Pose& goal_pose,
                                     bool diagonal_paths,
                                     nav_msgs::Path& result_path)
{
    result_path.poses.clear();
    nav_msgs::Path partial_path;
    geometry_msgs::Pose current_start = start_pose;

    for (const auto& via : via_poses) {
        nav_msgs::OccupancyGrid biased_cost_map = cost_map;
        geometry_msgs::Pose via_copy = via;
        PathPlanner::addViaPointBias(biased_cost_map, via_copy, 0.5, 400);
        partial_path.poses.clear();
        if (!PathPlanner::AStar(map, biased_cost_map, current_start, via_copy, diagonal_paths, partial_path)) return false;
        result_path.poses.insert(result_path.poses.end(), partial_path.poses.begin(), partial_path.poses.end());
        current_start = via_copy;
    }

    partial_path.poses.clear();
    if (!PathPlanner::AStar(map, cost_map, current_start, goal_pose, diagonal_paths, partial_path)) return false;
    result_path.poses.insert(result_path.poses.end(), partial_path.poses.begin(), partial_path.poses.end());
    return true;
}

nav_msgs::Path PathPlanner::SmoothPath(nav_msgs::Path& path, float weight_data, float weight_smooth, float tolerance)
{
    nav_msgs::Path newPath;
    for(int i=0; i< path.poses.size(); i++)
        newPath.poses.push_back(path.poses[i]);
    newPath.header.frame_id = "map";   
    if(path.poses.size() < 3)  return newPath;
    int attempts = 0;
    tolerance *= path.poses.size();
    float change = tolerance + 1;

    while(change >= tolerance && ++attempts < 1000)
    {
        change = 0;
        for(int i=1; i< path.poses.size() - 1; i++)
        {
            geometry_msgs::Point old_p = path.poses[i].pose.position;
            geometry_msgs::Point new_p = newPath.poses[i].pose.position;
            geometry_msgs::Point new_p_next = newPath.poses[i+1].pose.position;
            geometry_msgs::Point new_p_prev = newPath.poses[i-1].pose.position;
            float last_x = newPath.poses[i].pose.position.x;
            float last_y = newPath.poses[i].pose.position.y;
            new_p.x += weight_data*(old_p.x - new_p.x) + weight_smooth*(new_p_next.x + new_p_prev.x - 2.0*new_p.x);
            new_p.y += weight_data*(old_p.y - new_p.y) + weight_smooth*(new_p_next.y + new_p_prev.y - 2.0*new_p.y);
            change += fabs(new_p.x - last_x) + fabs(new_p.y - last_y);
            newPath.poses[i].pose.position = new_p;
        }
    }
    std::cout << "PathPlanner.->Smoothing finished after " << attempts << " attempts" <<  std::endl;
    return newPath;
}

Node::Node()
{
    this->index            = -1;
    this->g_value          = INT_MAX;
    this->f_value          = INT_MAX;
    this->in_open_list     = false;
    this->in_closed_list   = false;
    this->parent           = NULL;  
}

Node::~Node()
{
}
