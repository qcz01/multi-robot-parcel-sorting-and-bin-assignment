/**
 * @file edrp.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */



#pragma once


#include<algorithm>
#include<cmath>
#include<memory>
#include<queue>
#include<vector>
#include<unordered_map>
#include<unordered_set>
#include<functional>
#include <boost/functional/hash.hpp>
#include "task.h"
#include "search_utils.h"
#include "utils.h"



class EDRP{
public:
    EDRP(){};
    using Paths=std::vector<std::vector<Node>>;
    std::vector<std::vector<Node>> solve(MultiGoalTask& t, Graph& g,int horizon = std::numeric_limits<int>::max());

    
    std::pair<bool,bool> move(int robot_id,Paths&);        //recursively move robots

   
    

    std::map<int,int> priority;

    bool check_cycles(const std::vector<Node> &next_pos);

    
    

    std::vector<Node> next_step;

    bool isCyclicUtil(int robot_id,const std::vector<Node> &next_pos);

    std::vector<std::vector<Node>> future_paths;

    std::map<int,bool> cycling;
    std::unordered_map<Node,int> current_positions;
    std::set<int> recStack;
    std::map<int,bool> moved_robots;


    void debugger(int index);


    void delayRobot(int robot,Paths &paths);

    void moveRobot(int robot,Paths &paths);

    void move_all_robots_in_cycle(int robot,Paths &paths);

    int get_conflicted_robot(int robot,Paths &paths);



    int num_robots;

};