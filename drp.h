/**
 * @file drp.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-03
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
//distributed robust planner


class DRP{
public:
    DRP(){};

    std::vector<std::vector<Node>> solve(MultiGoalTask& t, Graph& g,int horizon = std::numeric_limits<int>::max());


    bool move(int robot_id,std::unordered_set<int> &moved,std::vector<std::vector<Node>>&);        //recursively move robots

    std::unordered_map<Node,int> current_positions;
    
    std::vector<std::pair<int,int>> vertex_conflicts;

    std::map<int,int> priority;

    bool check_cycles(const std::vector<Node> &next_pos);

    std::unordered_map<int,bool> moved_robots;
    std::set<int> recStack;

    std::vector<Node> next_step;

    bool isCyclicUtil(int robot_id,const std::vector<Node> &next_pos);

    std::vector<std::vector<Node>> future_paths;

    std::unordered_set<Node> occupied;

    int num_robots;
    



};