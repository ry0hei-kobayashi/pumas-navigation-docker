#include "PathPlanner.h"
#include <cmath>
#include <cstdlib>

//add by ryohei k 2024/6/29 for GrowObstacle
int CollisionLevel = 40;

//bool PathPlanner::AStar(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& startPose, geometry_msgs::Pose& goalPose, nav_msgs::Path& resultPath)
bool PathPlanner::AStar(nav_msgs::OccupancyGrid& map, nav_msgs::OccupancyGrid& cost_map, geometry_msgs::Pose& startPose, geometry_msgs::Pose& goalPose, nav_msgs::Path& resultPath)
{
    std::cout << "PathPlanner.-> Calculating by A* from " << startPose.position.x << "  ";
    std::cout << startPose.position.y << "  to " << goalPose.position.x << "  " << goalPose.position.y << std::endl;
    
    // Calculate start and goal cell indices
    int startCellX = (int)((startPose.position.x - map.info.origin.position.x) / map.info.resolution);
    int startCellY = (int)((startPose.position.y - map.info.origin.position.y) / map.info.resolution);
    int goalCellX = (int)((goalPose.position.x - map.info.origin.position.x) / map.info.resolution);
    int goalCellY = (int)((goalPose.position.y - map.info.origin.position.y) / map.info.resolution);
    int startCell = startCellY * map.info.width + startCellX;
    int goalCell = goalCellY * map.info.width + goalCellX;

    // Grow obstacles in the map
    //map = PathPlanner::GrowObstacles(map, 0.15);
    
    // Check if start or goal pose is inside occupied space
    if (map.data[goalCell] > CollisionLevel || map.data[goalCell] < 0) {
        std::cout << "PathPlanner.-> Cannot calculate path: goal point is inside occupied space" << std::endl;
        return false;
    }
    if (map.data[startCell] > CollisionLevel || map.data[startCell] < 0) {
        std::cout << "PathPlanner.-> Cannot calculate path: start point is inside occupied space" << std::endl;
        return false;
    }

    // Initialize arrays for A* algorithm
    bool* isKnown = new bool[map.data.size()];
    int* g_values = new int[map.data.size()];
    int* f_values = new int[map.data.size()];
    int* previous = new int[map.data.size()];
    bool* visited = new bool[map.data.size()];
    int* neighbors = new int[map.data.size()];
    int* nearnessToObstacles = new int[map.data.size()];
    std::vector<int> visitedAndNotKnown;
    
    int currentCell = startCell;

    // Calculate nearness to obstacles
    if (!PathPlanner::NearnessToObstacles(map, 0.6, nearnessToObstacles)) {
        std::cout << "PathPlanner.-> Cannot calculate nearness to obstacles u.u" << std::endl;
        return false;
    }

    // Initialize arrays
    for (int i = 0; i < map.data.size(); i++) {
        isKnown[i] = map.data[i] > CollisionLevel || map.data[i] < 0;
        g_values[i] = INT_MAX;
        f_values[i] = INT_MAX;
        previous[i] = -1;
        visited[i] = map.data[i] > CollisionLevel || map.data[i] < 0;
    }
    for (int i = 0; i < 8; i++)
        neighbors[i] = 0;

    // Set start node
    isKnown[currentCell] = true;
    g_values[currentCell] = 0;
    bool fail = false;
    int attempts = 0;

    // A* search loop
    while (currentCell != goalCell && !fail && attempts < map.data.size()) {
        // Calculate neighbors
        neighbors[0] = currentCell - map.info.width;
        neighbors[1] = currentCell - 1;
        neighbors[2] = currentCell + 1;
        neighbors[3] = currentCell + map.info.width;
        
        // Evaluate each neighbor
        for (int i = 0; i < 4; i++) {
            if (isKnown[neighbors[i]]) continue;
            
            // Calculate g_value (accumulated distance + nearness to obstacles)
            int g_value = g_values[currentCell] + 1 + nearnessToObstacles[neighbors[i]]; 
            
            // Calculate h_value (Manhattan distance to goal)
            int h_value_x = neighbors[i] % map.info.width - goalCellX;
            int h_value_y = neighbors[i] / map.info.width - goalCellY;
            int h_value = (int)(sqrt(h_value_x * h_value_x + h_value_y * h_value_y));
            
            // Update values if a better path is found
            if (g_value < g_values[neighbors[i]]) {
                g_values[neighbors[i]] = g_value;
                f_values[neighbors[i]] = g_value + h_value;
                previous[neighbors[i]] = currentCell;
            }
            
            // Mark neighbor as visited
            if (!visited[neighbors[i]])
                visitedAndNotKnown.push_back(neighbors[i]);
            
            visited[neighbors[i]] = true;
        }
        
        // Select next cell with minimum f_value
        int min_f_value_idx = -1;
        int min_f_value = std::numeric_limits<int>::max();
        
        for (int i = 0; i < visitedAndNotKnown.size(); i++) {
            if (f_values[visitedAndNotKnown[i]] < min_f_value) {
                min_f_value_idx = i;
                min_f_value = f_values[visitedAndNotKnown[i]];
            }
        }
        
        if (min_f_value_idx >= 0) {
            currentCell = visitedAndNotKnown[min_f_value_idx];
            isKnown[currentCell] = true;
            visitedAndNotKnown.erase(visitedAndNotKnown.begin() + min_f_value_idx);
        } else {
            fail = true;
        }
        
        attempts++;
    }
    
    // Check if path was found
    if (fail) {
        std::cout << "PathPlanner.-> Cannot find path to goal point by A* :'(" << std::endl;
        return false;
    }
    
    // Construct path in reverse order
    geometry_msgs::PoseStamped p;
    currentCell = goalCell;
    
    while (previous[currentCell] != -1) {
        p.pose.position.x = (currentCell % map.info.width) * map.info.resolution + map.info.origin.position.x;
        p.pose.position.y = (currentCell / map.info.width) * map.info.resolution + map.info.origin.position.y;
        p.pose.orientation.w = 1;
        p.header.frame_id = "map";
        resultPath.poses.insert(resultPath.poses.begin(), p);
        
        currentCell = previous[currentCell];
    }
    
    // Add start pose to path
    p.pose.position.x = (currentCell % map.info.width) * map.info.resolution + map.info.origin.position.x;
    p.pose.position.y = (currentCell / map.info.width) * map.info.resolution + map.info.origin.position.y;
    p.pose.orientation.w = 1;
    p.header.frame_id = "map";
    resultPath.poses.insert(resultPath.poses.begin(), p);

    // Clean up allocated memory
    delete[] isKnown;
    delete[] g_values;
    delete[] f_values;
    delete[] previous;
    delete[] neighbors;
    delete[] visited;

    std::cout << "PathPlanner.-> Resulting path by A* has " << resultPath.poses.size() << " points." << std::endl;
    return true;
}

/*
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
    idx_goal_y  = (int)((goal_pose.position.y  - map.info.origin.position.y)/map.info.resolution);
    idx_goal_x  = (int)((goal_pose.position.x  - map.info.origin.position.x)/map.info.resolution); 
    int idx_start = idx_start_y*map.info.width + idx_start_x; //startCell
    int idx_goal  = idx_goal_y *map.info.width + idx_goal_x; //goalCell

    //double distance = 0.1;
    //double angle = 1.414213562;
    //

    map = PathPlanner::GrowObstacles(map, 0.15);


    //if(map.data[idx_goal] != 0)
    if(map.data[idx_goal] > CollisionLevel || map.data[idx_goal] < 0)
    {
        std::cout << "PathPlanner.->Goal point is inside non-free space!!!!" << std::endl;


	//double new_goal_x = goal_pose.position.x - distance * sin(angle);
	//double new_goal_y = goal_pose.position.y - distance * cos(angle);
        //distance ++;
	//goal_pose.position.x = new_goal_x;
	//goal_pose.position.y = new_goal_y;

        //// Recalculate the goal indices
        //idx_goal_x  = (int)((goal_pose.position.x  - map.info.origin.position.x)/map.info.resolution);
        //idx_goal_y  = (int)((goal_pose.position.y  - map.info.origin.position.y)/map.info.resolution);
        //idx_goal = idx_goal_y * map.info.width + idx_goal_x;

        return false;
    }

    if(map.data[idx_start] > CollisionLevel || map.data[idx_start] < 0)
    {
        std::cout << "PathPlanner.->Start point is inside non-free space!!!!" << std::endl;
        return false;
    }
    
    
    //add by ryoheik for NearnessToObstacle Function
    //std::cout << "Creating arrays for dijkstra data" << std::endl;
    bool* isKnown = new bool[map.data.size()];
    int* g_values = new int[map.data.size()];
    int* f_values = new int[map.data.size()];
    int* previous = new int[map.data.size()];
    bool* visited = new bool[map.data.size()];
    int* neighbors = new int[map.data.size()];
    //int* waveFrontPotentials = new int[map.data.size()];
    int* nearnessToObstacles = new int[map.data.size()];
    std::vector<int> visitedAndNotKnown;

    if(!PathPlanner::NearnessToObstacles(map, 0.6, nearnessToObstacles))
    {
        std::cout << "PathPlanner.->Cannot calculate nearness to obstacles u.u" << std::endl;
        return false;
    }
    //add by ryoheik for NearnessToObstacle Function
    
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
                //if(map.data[node_neighbors[i]] != 0 || nodes[node_neighbors[i]].in_closed_list)

		//add by R kobayashi
		int neighbor_index = node_neighbors[i];

                float delta_g = i < 4 ? 1.0 : 1.414213562;
                float g_value = current_node->g_value + (i < 4 ? 1.0 : 1.414213562) + cost_map.data[node_neighbors[i]];
                float h_value;
                int   h_value_x = node_neighbors[i]%map.info.width - idx_goal_x;
                int   h_value_y = node_neighbors[i]/map.info.width - idx_goal_y;

                if(map.data[node_neighbors[i]] <  0 || neighbor_index >= static_cast<int>(map.data.size())){ //nodes[node_neighbors[i]].in_closed_list)
                    continue;
		}

                if (map.data[neighbor_index] != 0 || nodes[neighbor_index].in_closed_list) {
	            continue;
	        }
           
                //Node* neighbor = &nodes[node_neighbors[i]];
                Node* neighbor = &nodes[neighbor_index];

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
*/

//add by ryohei k 2024/06/29
nav_msgs::OccupancyGrid PathPlanner::GrowObstacles(nav_msgs::OccupancyGrid& map, float growDist)
{
    //HAY UN MEGABUG EN ESTE ALGORITMO PORQUE NO ESTOY TOMANDO EN CUENTA QUE EN LOS BORDES DEL
    //    //MAPA NO SE PUEDE APLICAR CONECTIVIDAD CUATRO NI OCHO. FALTA RESTRINGIR EL RECORRIDO A LOS BORDES MENOS UNO.
    //        //POR AHORA FUNCIONA XQ CONFÍO EN QUE EL MAPA ES MUCHO MÁS GRANDE QUE EL ÁREA REAL DE NAVEGACIÓN
    if(growDist <= 0)
    {
        std::cout << "PathPlanner.->Cannot grow map. Grow dist must be greater than zero." << std::endl;
        return map;
    }
    nav_msgs::OccupancyGrid newMap = map;

    int growSteps = (int)(growDist / map.info.resolution);
    int boxSize = (2*growSteps + 1) * (2*growSteps + 1);
    int* neighbors = new int[boxSize];
    int counter = 0;

    //std::cout << "PathPlanner.->Growing map " << growSteps << " steps" << std::endl;
    for(int i=-growSteps; i<=growSteps; i++)
        for(int j=-growSteps; j<=growSteps; j++)
        {
            neighbors[counter] = j*map.info.width + i;
            counter++;
        }
        /*
        std::cout << "Calculation of neighbors finished: " << std::endl;
        for(int i=0; i <boxSize; i++)
            std::cout << neighbors[i] << std::endl;
        */
    int startIdx = growSteps*map.info.width + growSteps;
    int endIdx = map.data.size() - growSteps*map.info.width - growSteps;

    if(endIdx <= 0)
    {
        std::cout << "PathPlanner.->Cannot grow map. Map is smaller than desired growth." << std::endl;
        return map;
    }

    for(int i=startIdx; i < endIdx; i++)
        if(map.data[i] > CollisionLevel) //Then, is an occupied cell
            for(int j=0; j < boxSize; j++) //If it is occupied, mark as occupied all neighbors in the neighbor-box
                newMap.data[i+neighbors[j]] = 100;
    delete[] neighbors;
    
    //std::cout << "PathPlanner.->Map-growth finished." << std::endl;
    return newMap;
}

//add by ryohei k 2024/06/29
bool PathPlanner::NearnessToObstacles(nav_msgs::OccupancyGrid& map, float distOfInfluence, int*& resultPotentials)
{
    //This function calculates the "nearness to obstacles", e.g., for the following grid:
    /*
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 3 3 3 3 3 2 1 0 1 1 1 1 1
      0 0 x x x 0 0 0 0 0 0 0 0 0                           2 3 x x x 3 2 1 0 1 2 2 2 2
      0 0 x x 0 0 0 0 0 0 0 0 0 0   the resulting nearness  2 3 x x 3 3 2 1 0 1 2 3 3 3
      0 0 x x 0 0 0 0 0 0 0 0 x x   values would be:        2 3 x x 3 2 2 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 x x                           2 3 3 3 3 2 1 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 2 2 2 2 2 1 0 0 1 2 3 3 3
      Max nearness value will depend on the distance of influence. 
     */
    if(distOfInfluence < 0)
    {
        std::cout << "PathPlanner.->Cannot calc brushfire. DistOfIncluence must be greater than zero." << std::endl;
    }
    if(resultPotentials == 0)
    {
        std::cout << "PathPlanner.->Cannot calc brushfire. 'resultPotentials' param must be not null." << std::endl;
        return false;
    }
    
    int steps = (int)(distOfInfluence / map.info.resolution);
    //std::cout << "PathPlanner.->Calculating nearness with " << steps << " steps. " << std::endl;
    
    int boxSize = (steps*2 + 1) * (steps*2 + 1);
    int* distances = new int[boxSize];
    int* neighbors = new int[boxSize];
    int startIdx = steps*map.info.width + steps;
    int endIdx = map.data.size() - steps*map.info.width - steps;
    if(endIdx <= 0)
    {
        std::cout << "PathPlanner.->Cannot calc brushfire. There is an error in index calculation. Sorry." << std::endl;
        return false;
    }
    int counter = 0;
    for(int i=-steps; i<=steps; i++)
        for(int j=-steps; j<=steps; j++)
        {
            neighbors[counter] = i*map.info.width + j;
            distances[counter] = (steps - std::max(std::abs(i), std::abs(j)) + 1)/2; //Use value/2 just for getting a smaller value
            counter++;
        }

    //std::cout << "Nearness values to be used: " << std::endl;
    //for(int i=0; i < boxSize; i++)
    //    std::cout << distances[i] << " ";
    //std::cout << std::endl;

    for(int i=0; i < map.data.size(); i++)
        resultPotentials[i] = 0;
    
    for(int i=startIdx; i < endIdx; i++)
        if(map.data[i] > CollisionLevel)
            for(int j = 0; j < boxSize; j++)
                if(resultPotentials[i+neighbors[j]] < distances[j])
                    resultPotentials[i+neighbors[j]] = distances[j];

    delete[] distances;
    delete[] neighbors;
    //std::cout << "PathPlanner.->Finished, calculation of nearness to obstacles :D" << std::endl;
    return true;
}

//no diff from erasers_nav
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
    //this->index            = -1;
    //this->g_value          = INT_MAX;
    //this->f_value          = INT_MAX;
    //this->in_open_list     = false;
    //this->in_closed_list   = false;
    this->parent           = NULL;  
}

Node::~Node()
{
}
