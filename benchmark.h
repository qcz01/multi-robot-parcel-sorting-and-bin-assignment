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