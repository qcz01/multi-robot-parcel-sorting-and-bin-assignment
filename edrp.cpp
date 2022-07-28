#include"edrp.h"
#include "a_star.hpp"



std::vector<std::vector<Node>> EDRP::solve(MultiGoalTask& task, Graph& g,int horizon){
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
    // auto path_planner=AStarEps(g);
    std::vector<size_t> goal_ids(task.num_robots,0);


    for (size_t i = 0; i < task.num_robots; i++) {
        auto robot_index = robot_order[i];
        // future_paths[robot_order[i]] = path_planner.reversed_search(
        //     task.starts[robot_order[i]], task.goals[robot_order[i]][0]);
        future_paths[robot_order[i]]=path_planner.reversed_search_focal( task.starts[robot_order[i]], task.goals[robot_order[i]][0],i,future_paths);
        // if (suo) suo->add_path(future_paths[robot_order[i]], true);
        future_paths[robot_order[i]].pop_back();
        // std::cout<<future_paths[robot_order[i]].size()<<std::endl;
    }
    







    while (true) {
        // Check if robots are already at the goals. If goals reached,
        // return the solution; if max horizon reached, return solution.
        bool future_paths_all_empty = true;
 

        for (size_t i = 0; i < task.num_robots; i++){
            // if(future_paths[i].back()==Node(-1,-1))future_paths[i].pop_back();
            if (!future_paths[i].empty() || goal_ids[i]<task.goals[i].size()) {
                future_paths_all_empty = false;
                break;
            }
        }
        
        if (future_paths_all_empty || paths.size() > horizon) {//found the solutions
            task.target_goal_reaching_num=0;
            for(size_t i=0;i<goal_ids.size();i++){
                task.target_goal_reaching_num+=goal_ids[i];
                // std::cout<<"robot "<<i<<" completed "<<goal_ids[i]<<" tasks"<<std::endl;

            }
            task.finished_goals=goal_ids;
            // check_multi_goal_path_feasibility(paths,task,g);
            // save_paths_as_txt("./text.txt",paths);
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
                    // future_paths[i] = path_planner.reversed_search(start_i, goal_i);
                
                    future_paths[i] = path_planner.reversed_search_focal(start_i, goal_i,i,future_paths);
                    // std::cout<<" successfully done "<<std::endl;
                   
                    future_paths[i].pop_back();
                   
                }
     
                // if (suo)
                //     suo->add_path(future_paths[i], true);
            }
        }

        // auto next_step = std::vector<Node>(task.num_robots);
        next_step.clear();
        current_positions.clear();
        moved_robots.clear();
        // std::unordered_set<Node> current_vertices;
        for (size_t i = 0; i < task.num_robots; i++){
            // next_step[i] = future_paths[i].back();

            // assert(future_paths[i].size()!=0);
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
   
        recStack.clear();
        cycling.clear();
        paths.push_back(paths.back());
        // for(int i=0;i<task.num_robots;i++){
        //     std::cout<<" robot "<<i<<" current step ="<<paths.back()[i]<<" next step "<<next_step[i]<<std::endl;
        // if(paths.size()==2){
        //     std::cout<<paths.back()[12]<<"  "<<future_paths[12].back()<<std::endl;
        //     std::cout<<paths.back()[19]<<"  "<<future_paths[19].back()<<std::endl;
        //     std::cout<<paths.back()[4]<<"  "<<future_paths[4].back()<<std::endl;
        //     std::cout<<paths.back()[3]<<"  "<<future_paths[3].back()<<std::endl;
        //     std::cout<<paths.back()[2]<<"  "<<future_paths[2].back()<<std::endl;
        //     std::cout<<paths.back()[1]<<"  "<<future_paths[1].back()<<std::endl;
        //     // exit(0);
        // }
        // }

        for(int i=0;i<task.num_robots;i++){
            // if(i==88 ) std::cout<<" robot "<<i<<" current step ="<<paths.back()[i]<<" next step "<<next_step[i]<<"  future paths[i].size()="<<future_paths[i].size()<<"  goal_id= "<<goal_ids[i]<<std::endl;
   
            recStack.clear();
            move(i,paths);
        }
    }
       
}

void EDRP::move_all_robots_in_cycle(int robot,Paths&paths){
    std::set<int> visited;
    auto curr=robot;
    // std::cout<<"Move along a cycle"<<std::endl;
    bool debbuged=false;
    if(paths.size()==2) debbuged=true;
    while(true){
  
        if(visited.find(curr)!=visited.end()) break;
        visited.insert(curr);
        auto v=future_paths[curr].back();
        if(debbuged)std::cout<<"oh cycled "<<curr<<" " <<v <<std::endl;
        moveRobot(curr,paths);
        cycling[curr]=true;
        curr=current_positions[v];
    }

}


int EDRP::get_conflicted_robot(int r,Paths &paths){
    auto ur=paths.back()[r];
    auto vr=future_paths[r].back();
    for(int x=ur.x-1;x<=ur.x+1;x++){
        for(int y=ur.y-1;y<=ur.y+1;y++){
            Node nb=Node(x,y);
            if(nb==ur) continue;
            if(current_positions.find(nb)==current_positions.end()) continue;
            auto rk=current_positions[nb];
            if(moved_robots.find(rk)!=moved_robots.end() and paths.back()[rk]==vr)return rk;
            auto vk=future_paths[rk].back();
            if(vk==vr) return rk;

        }
    }
    return -1;
}

std::pair<bool,bool> EDRP::move(int robot,Paths& paths){
    bool debugged=false;
    if(paths.size()==2) debugged=true;
    if(robot<0) return {true,true};
    auto next_v=next_step[robot];
    if(moved_robots.find(robot)!=moved_robots.end())return {moved_robots[robot],cycling[robot]};  //aleady moved
    if(next_v==Node(-1,-1)){
        paths.back()[robot]=next_v;
        future_paths[robot].pop_back();
        // assert(future_paths[robot].size()<=100);
        return {true,false};
    }
    if(recStack.find(robot)!=recStack.end()){
        // if(paths.size()==112) {
        //     std::cout<<"find cycles "<<robot<<std::endl;
        //     exit(0);
        // }
        move_all_robots_in_cycle(robot,paths);
        return {true,true};
    }
    recStack.insert(robot);
    int robot_j;
    if(current_positions.find(next_v)==current_positions.end())robot_j=-1;
    else robot_j=current_positions[next_v];
    // if(robot==2 and debugged==true) std::cout<<"rj="<<robot_j<<std::endl;
    auto flag=move(robot_j,paths);
    if(moved_robots.find(robot)!=moved_robots.end()){
        if(debugged and robot==12){
            std::cout<<"moved in this cycle  "<<paths.back()[12]<<std::endl;
        }
        return {moved_robots[robot],cycling[robot]};
    }
    
    if(flag.first==false){
        delayRobot(robot,paths);
        return {false,false};
    }
    int rk=get_conflicted_robot(robot,paths);

    if(rk==12 and debugged==true) std::cout<<"conflicted "<<rk<<"  "<<robot<<std::endl;

    if(rk==-1){
        moveRobot(robot,paths);
        return {true,false};
    }

    if(future_paths[rk].size()>future_paths[robot].size() or cycling.find(rk)!=cycling.end()){
        if(cycling.find(rk)==cycling.end())
            moveRobot(rk,paths);
        delayRobot(robot,paths);
        return {false,false};
    }else{
        delayRobot(rk,paths);
        moveRobot(robot,paths);
        return {true,false};
    }

    

    
}

void EDRP::moveRobot(int i,Paths &paths){
    moved_robots[i]=true;
    assert(future_paths[i].empty()==false);
    paths.back()[i]=future_paths[i].back();
    future_paths[i].pop_back();
}

void EDRP::delayRobot(int i, Paths &paths){
    moved_robots[i]=false;
    paths.back()[i]=paths.back()[i];
}


bool EDRP::check_cycles(const std::vector<Node> &next_pos){
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


bool EDRP::isCyclicUtil(int robot_id,const std::vector<Node> &next_pos){
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