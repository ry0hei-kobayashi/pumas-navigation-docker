bool PathPlanner::AStar(nav_msgs::OccupancyGrid& map, nav_msgs::OccupancyGrid& cost_map,
                      geometry_msgs::Pose& start_pose, geometry_msgs::Pose& goal_pose, bool diagonal_paths, nav_msgs::Path& result_path)
{
    std::cout << "PathCalculator.-> Calculating by A* from " << start_pose.position.x << "  ";
    std::cout << start_pose.position.y << "  to " << goal_pose.position.x << "  " << goal_pose.position.y << std::endl;

    int idx_start_x;
    int idx_start_y;
    int idx_goal_x;
    int idx_goal_y;
    idx_start_y = (int)((start_pose.position.y - map.info.origin.position.y) / map.info.resolution);
    idx_start_x = (int)((start_pose.position.x - map.info.origin.position.x) / map.info.resolution);
    int idx_start = idx_start_y * map.info.width + idx_start_x;

    idx_goal_y  = (int)((goal_pose.position.y - map.info.origin.position.y) / map.info.resolution);
    idx_goal_x  = (int)((goal_pose.position.x - map.info.origin.position.x) / map.info.resolution);
    int idx_goal  = idx_goal_y * map.info.width + idx_goal_x;

    // フラッドフィル法によるゴール位置の調整
    if (map.data[idx_goal] != 0)  // ゴールが障害物にある場合
    {
        std::queue<std::pair<int, int>> search_queue;
        std::vector<std::vector<bool>> visited(map.info.height, std::vector<bool>(map.info.width, false));

        search_queue.push(std::make_pair(idx_goal_x, idx_goal_y));
        visited[idx_goal_y][idx_goal_x] = true;

        // 隣接するセルの探索に使用する方向（上下左右、および対角方向も考慮）
        std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                                       {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        bool found_free_cell = false;
        while (!search_queue.empty() && !found_free_cell)
        {
            auto [current_x, current_y] = search_queue.front();
            search_queue.pop();

            // 隣接するセルをチェック
            for (auto& dir : directions)
            {
                int next_x = current_x + dir.first;
                int next_y = current_y + dir.second;

                // マップの範囲外を除外
                if (next_x < 0 || next_x >= map.info.width || next_y < 0 || next_y >= map.info.height)
                    continue;

                if (visited[next_y][next_x])
                    continue;

                visited[next_y][next_x] = true;
                int next_idx = next_y * map.info.width + next_x;

                // 自由なセルを見つけたら、その位置をゴールに設定
                if (map.data[next_idx] == 0)
                {
                    idx_goal_x = next_x;
                    idx_goal_y = next_y;
                    idx_goal = next_idx;
                    found_free_cell = true;
                    break;
                }

                // 次のセルを探索するためにキューに追加
                search_queue.push(std::make_pair(next_x, next_y));
            }
        }

        // 自由なセルが見つからない場合
        if (!found_free_cell)
        {
            ROS_ERROR("PathPlanner.->Unable to find free space near goal!!!!");
            return false;
        }
    }

    // スタートが非自由空間にある場合のエラーハンドリング
    if (map.data[idx_start] != 0)
    {
        ROS_ERROR("PathPlanner.->Start point is inside non-free space!!!!");
        return false;
    }

    std::vector<Node> nodes;
    Node* current_node; 
    std::vector<int> node_neighbors;
    int steps = 0;
    nodes.resize(map.data.size());
    if (diagonal_paths)
        node_neighbors.resize(8);
    else
        node_neighbors.resize(4);
    std::priority_queue<Node*, std::vector<Node*>, CompareByFValue> open_list;
    for (size_t i = 0; i < map.data.size(); i++)
        nodes[i].index = i;

    current_node = &nodes[idx_start];
    current_node->g_value = 0;
    current_node->in_open_list = true;    
    open_list.push(current_node);
    
    while (!open_list.empty() && current_node->index != idx_goal)
    {
        current_node = open_list.top();  
        open_list.pop();                   
        current_node->in_closed_list = true;

        // 隣接ノードの計算
        node_neighbors[0] = current_node->index + map.info.width;
        node_neighbors[1] = current_node->index + 1;
        node_neighbors[2] = current_node->index - map.info.width;
        node_neighbors[3] = current_node->index - 1;
        if (diagonal_paths)
        {
            node_neighbors[4] = current_node->index + map.info.width + 1;
            node_neighbors[5] = current_node->index + map.info.width - 1;
            node_neighbors[6] = current_node->index - map.info.width + 1;
            node_neighbors[7] = current_node->index - map.info.width - 1;
        }
        
        // 隣接ノードに対する処理
        for (size_t i = 0; i < node_neighbors.size(); i++)
        {
            // 境界チェックを追加
            if (node_neighbors[i] < 0 || node_neighbors[i] >= map.data.size() || map.data[node_neighbors[i]] != 0 || nodes[node_neighbors[i]].in_closed_list)
                continue;

            Node* neighbor = &nodes[node_neighbors[i]];
            float delta_g = i < 4 ? 1.0 : 1.414213562;
            float g_value = current_node->g_value + delta_g + cost_map.data[node_neighbors[i]];
            float h_value;
            int h_value_x = node_neighbors[i] % map.info.width - idx_goal_x;
            int h_value_y = node_neighbors[i] / map.info.width - idx_goal_y;
            if (diagonal_paths)
                h_value = sqrt(h_value_x * h_value_x + h_value_y * h_value_y);
            else
                h_value = fabs(h_value_x) + fabs(h_value_y);
            
            if (g_value < neighbor->g_value)
            {
                neighbor->g_value = g_value;
                neighbor->f_value = g_value + h_value;
                neighbor->parent  = current_node;
            }

            if (!neighbor->in_open_list)
            {
                neighbor->in_open_list = true;
                open_list.push(neighbor);
            }
        }
        steps++;
    }
    std::cout << "PathPlanner.->A* Algorithm ended after " << steps << " steps" << std::endl;

    // ゴールに到達できなかった場合
    if (current_node->index != idx_goal)
        return false;
    
    // 結果のパスを生成
    result_path.header.frame_id = "map";
    result_path.poses.clear();
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    while (current_node->parent != NULL)
    {
        p.pose.position.x = current_node->index % map.info.width * map.info.resolution + map.info.origin.position.x;	
        p.pose.position.y = current_node->index / map.info.width * map.info.resolution + map.info.origin.position.y;
        result_path.poses.insert(result_path.poses.begin(), p);
        current_node = current_node->parent;
    }
    
    std::cout << "PathCalculator.->Resulting path by A* has " << result_path.poses.size() << " points." << std::endl;
    return true;
}

