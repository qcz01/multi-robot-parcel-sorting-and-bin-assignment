/**
 * @file benchmark.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include"task.h"
#include<iostream>
#include"drp.h"

#include <experimental/filesystem>

std::pair<std::string,std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>>
get_warehouse_multi_goal_tasks_small();

void test_multi_goal_drp();
typedef std::pair<
    std::string, std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>> (
    *MultiTaskFunction)(void);

void multi_goal_test_helper_drp(MultiTaskFunction tf, int horizon);


MultiGoalTask get_multi_goal_task_(std::vector<Node> &station_positions,std::vector<std::vector<double>>&probability, std::map<int,std::vector<Node>>&type_bin,int num_robots,int num_goals_per_robot,int target_goal_reaching_num, Graph& g);


Node get_closest_bin(std::vector<Node> &station_position,std::map<int,std::vector<Node>> &bin_map,int station,int type);

void read_settings_from_txt(std::string file_name,std::map<int,std::vector<Node>>&type_bin,std::vector<Node>&station_positions);