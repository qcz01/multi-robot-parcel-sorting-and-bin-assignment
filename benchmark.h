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
#include "ecbs_lifelong.h"
#include <experimental/filesystem>
#include "ddm.h"

std::pair<std::string,std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>>
get_warehouse_multi_goal_tasks_small();


void test_multi_goal_drp();
typedef std::pair<
    std::string, std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>> (
    *MultiTaskFunction)(void);







Node get_closest_bin(std::vector<Node> &station_position,std::map<int,std::vector<Node>> &bin_map,int station,int type);


void multi_goal_test(MultiGoalTask &,int);


struct Warehouse_Infos{
    std::map<int,std::vector<Node>>bin_assignment;
    std::vector<Node>station_positions;
    int xmax;
    int ymax;
    std::vector<std::vector<double>>probability;
};

Warehouse_Infos read_settings_from_json(std::string file_name);

MultiGoalTask get_multi_goal_task_prob(Warehouse_Infos &settings,int num_robots,int num_goals_per_robot,int target_goal_reaching_num, Graph& g);


std::pair<std::string,std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>>
get_sorting_center_tasks(Warehouse_Infos &settings);


std::pair<std::string,std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>>
get_sorting_center_tasks_undirected(Warehouse_Infos &settings);


// void multi_goal_test_helper_drp(MultiTaskFunction tf, int horizon);
// void test_sorting_center_drp(std::string file_name);

void test_multi_goal_drp(std::string file_name);
void test_multi_goal_ecbs(std::string file_name);
void test_multi_goal_ecbs_directed(std::string file_name);
void test_multi_goal_edrp(std::string file_name);

typedef std::pair<std::string,std::vector<std::pair<ECBSSolver, std::string>>> (*ECBSSolverFunction)(void);

typedef std::pair<std::string,std::vector<std::pair<DdmSolver, std::string>>> (*DdmSolverFunction)(void);

// void multi_goal_test_helper(MultiTaskFunction tf, ECBSSolverFunction sf,int horizon,Warehouse_Infos &settings);

void test_multi_goal_ddm(std::string file_name);

void save_result_as_json(MultiGoalTask &task,std::string file_name,std::vector<std::vector<Node>>&paths);
// void multi_goal_test_helper_ddm(MultiTaskFunction tf,int horizon,Warehouse_Infos &settings);

void debug(std::string file_name);


void test_ORCA(std::string file_name);