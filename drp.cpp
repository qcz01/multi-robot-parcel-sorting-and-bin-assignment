#include"drp.h"



std::vector<std::vector<Node>> DRP::solve(MultiGoalTask& task, Graph& g,int horizon){
    // std::cout<<"MAPF horizon="<<horizon<<std::endl;
    srand(0);
    auto paths =
        std::vector<std::vector<Node>>();  // Valid solution paths. Arranged as
                                           // config at time. Will be populated
                                           // as the algorithm runs
    paths.push_back(std::vector<Node>());
    for (size_t i = 0; i < task.num_robots; i++)
        paths[0].push_back(task.starts[i]);
    num_robots=task.num_robots;
    future_paths = std::vector<std::vector<Node>>(
        task.num_robots,
        std::vector<Node>());  // Envisioned paths without
                               // considering collisions. Arranged
                               // as a path per robot. Note that
                               // the paths are arranged in
                               // reverse for quick pop operation

    auto robot_order = std::vector<size_t>(task.num_robots);
    auto path_lengths = std::vector<size_t>(task.num_robots);
    for (size_t i = 0; i < task.num_robots; i++) {
        robot_order[i] = i;
        path_lengths[i] = get_manhattan_distance(task.starts[i], task.goals[i][0]);
    }
    std::sort(robot_order.begin(), robot_order.end(),
              [&path_lengths](size_t a, size_t b) {
                  return path_lengths[a] < path_lengths[b];
              });
    // First, get an independent path for each robot
    auto path_planner =
        SingleRobotPathPlanner<OmniDirectionalRobot::State,
                               OmniDirectionalRobot::Action,
                               OmniDirectionalRobot::Environment>(g);

    std::vector<size_t> goal_ids(task.num_robots,0);


    for (size_t i = 0; i < task.num_robots; i++) {
        auto robot_index = robot_order[i];
        future_paths[robot_order[i]] = path_planner.reversed_search(
            task.starts[robot_order[i]], task.goals[robot_order[i]][0]);
        // if (suo) suo->add_path(future_paths[robot_order[i]], true);
        future_paths[robot_order[i]].pop_back();
        // std::cout<<future_paths[robot_order[i]].size()<<std::endl;
    }




    // exit(0);



    while (true) {
        // Check if robots are already at the goals. If goals reached,
        // return the solution; if max horizon reached, return solution.
        bool future_paths_all_empty = true;


        for (size_t i = 0; i < task.num_robots; i++){
            if (!future_paths[i].empty() || goal_ids[i]<task.goals[i].size()) {
                future_paths_all_empty = false;
               
                break;
            }
        }
        if (future_paths_all_empty || paths.size() > horizon) {//found the solutions
            task.target_goal_reaching_num=0;
            for(size_t i=0;i<goal_ids.size();i++){
                task.target_goal_reaching_num+=goal_ids[i];
            }
            // std::cout<<future_paths_all_empty<<"   "<<paths.size()<<"   "<<horizon<<"  "<<goal_ids[0]<<"   "<<task.goals[0].size()<<std::endl;
            return paths;
        }

        for (size_t i = 0; i < task.num_robots; i++){
            // std::cout<<"debug "<<future_paths[i].size()<<"  current goal id="<<goal_ids[i]<<" task number="<<task.goals[i].size()<<std::endl;
            if (future_paths[i].empty())
            {
                // std::cout<<"empty "<<std::endl;
                if (goal_ids[i] >= task.goals[i].size())
                {
                    future_paths[i].push_back(Node(-1,-1)); // all the goals are finished,remove this robot
                }
                else
                {
                    while (task.goals[i][goal_ids[i]] == paths.back()[i]){
                        // std::cout<<"Next goal"<<std::endl;
                        goal_ids[i]++; // new goal assigned to robot i
                    }
                    if(goal_ids[i]>=task.goals[i].size()){
                        future_paths[i].push_back(Node(-1,-1));
                        continue;
                    }
                    // std::cout<<"robot "<<i<<" is planning task "<<goal_ids[i]<<std::endl;
                    auto start_i = paths.back()[i];
                    auto goal_i = task.goals[i][goal_ids[i]];
                    // std::cout<<"searching for "<<i<<" "<<start_i<<" to  "<<goal_i<<"  "<<goal_ids[i]<<"  "<<task.goals[i].size()<<std::endl;
                    future_paths[i] = path_planner.reversed_search(start_i, goal_i);
                   
                    future_paths[i].pop_back();
                }
     
                // if (suo)
                //     suo->add_path(future_paths[i], true);
            }
        }

        // auto next_step = std::vector<Node>(task.num_robots);
        next_step.clear();
        current_positions.clear();
        // std::unordered_set<Node> current_vertices;
        for (size_t i = 0; i < task.num_robots; i++){
            // next_step[i] = future_paths[i].back();
            next_step.push_back(future_paths[i].back());
            // current_vertices.insert(paths.back()[i]);
            current_positions.insert({paths.back()[i],i});
        }

        // for(auto [v,id]:current_positions){
        //     std::cout<<v<<"  "<<id<<std::endl;
        // }
        // std::cout<<"before "<<next_step[0]<<std::endl;
        // if(check_cycles(next_step)==true){
        //     std::cout<<"FOund Cycles!"<<std::endl;
        // }
        // std::cout<<"after "<<next_step[0]<<std::endl;
        // for(auto [v,id]:current_positions){
        //     std::cout<<v<<"  "<<id<<std::endl;
        // }
        // exit(0);
        std::unordered_set<int> moved;
        recStack.clear();
        occupied.clear();
        paths.push_back(paths.back());
        for(int i=0;i<task.num_robots;i++){
            move(i,moved,paths);
        }
    }
    //     
}

bool DRP::move(int robot,std::unordered_set<int> &moved,std::vector<std::vector<Node>>& paths){
    auto next_v=next_step[robot];
    if(moved.find(robot)!=moved.end())return true;//aleady moved
    if(current_positions.find(next_v)==current_positions.end()){    //next v is not occupied by other robots
       
        for(int j=robot+1;j<num_robots;j++){
            auto next_vj=next_step[j];
            if(next_v==next_vj){
                if(future_paths[robot].size()<future_paths[j].size()){
                    paths.back()[robot]=future_paths[robot].back();
                    future_paths[robot].pop_back();
                    occupied.insert(future_paths[robot].back());
                    occupied.insert(paths.back()[j]);
                    moved.insert(robot);
                    moved.insert(j);
                    return true;
                  
                }
                else{
                    paths.back()[j]=future_paths[j].back();
                    future_paths[j].pop_back();
                    occupied.insert(future_paths[j].back());
                    occupied.insert(paths.back()[robot]);
                    moved.insert(robot);
                    moved.insert(j);
                    return false;
                }
            }
        }
    
        paths.back()[robot]=future_paths[robot].back();
        future_paths[robot].pop_back();
        moved.insert(robot);
        occupied.insert(future_paths[robot].back());
        return true;
                    
        
    }else{      //recursively ask its front robot
        recStack.insert(robot);
        auto next_robot=current_positions[next_v];
        if(recStack.find(next_robot)!=recStack.end())//a cycle is found
        {
            paths.back()[robot]=future_paths[robot].back();
            moved.insert(robot);
            occupied.insert(paths.back()[robot]);
            return true;
        }
        bool flag=move(next_robot,moved,paths);
        if(flag==false){
            //delay this robot
            moved.insert(robot);
            occupied.insert(paths.back()[robot]);
            return false;

        }else{
            for (int j = robot + 1; j < num_robots; j++)
            {
                auto next_vj = next_step[j];
                if (next_v == next_vj)
                {
                    if (future_paths[robot].size() < future_paths[j].size())
                    {
                        paths.back()[robot] = future_paths[robot].back();
                        future_paths[robot].pop_back();
                        occupied.insert(future_paths[robot].back());
                        occupied.insert(paths.back()[j]);
                        moved.insert(robot);
                        moved.insert(j);
                        return true;
                    }
                    else
                    {
                        paths.back()[j] = future_paths[j].back();
                        future_paths[j].pop_back();
                        occupied.insert(future_paths[j].back());
                        occupied.insert(paths.back()[robot]);
                        moved.insert(robot);
                        moved.insert(j);
                        return false;
                    }
                }
            }
        }
    }
}


bool DRP::check_cycles(const std::vector<Node> &next_pos){
    int num_robots=next_pos.size();

    moved_robots.clear();  
    recStack.clear();
    for(int i=0;i<num_robots;i++){
        if(moved_robots.find(i)==moved_robots.end()){
            if(isCyclicUtil(i,next_pos)==true) return true;
        }
    }
    return false;
}


bool DRP::isCyclicUtil(int robot_id,const std::vector<Node> &next_pos){
    moved_robots.insert({robot_id,true});
    recStack.insert(robot_id);
    auto next_v=next_pos[robot_id];
    if(current_positions.find(next_v)==current_positions.end()) return false;
    auto next_robot=current_positions[next_v];
    if(moved_robots.find(next_robot)==moved_robots.end()){
        if(isCyclicUtil(next_robot,next_pos)==true) return true;
        else if(recStack.find(next_robot)!=recStack.end()) return  true;
    }
    recStack.erase(robot_id);
    return false;



}