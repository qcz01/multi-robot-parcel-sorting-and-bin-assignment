/**
 * @file ecbs.h
 * @author Shuai Han (shuai.han@rutgers.edu)
 * @brief the ECBS caller (https://github.com/whoenig/libMultiRobotPlanning),
 * designed for solving longer horizon problems
 * @version 0.1
 * @date 2021-01-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

// #include <algorithm>
// #include <cmath>
// #include <fstream>
// #include <limits>
// #include <random>
// #include <sstream>
// #include <string>
// #include <unordered_map>
// #include <std::vector>

#include "custom_astar.h"
#include "ecbs_utils.h"
#include "search_utils.h"
#include "suo.h"
#include "task.h"
#include "utils.h"

inline std::vector<std::vector<Node>> solve_ecbs(Graph& graph,
                                                 std::vector<Node>& starts,
                                                 std::vector<Node>& goals,
                                                 bool low_level = false) {
    assert(starts.size() == goals.size());
    // Setup searching environment
    auto ecbs_starts = std::vector<ECBSState>();
    auto ecbs_goals = std::vector<Node>();
    for (size_t i = 0; i < starts.size(); i++) {
        ecbs_starts.push_back(ECBSState(0, starts[i]));
        ecbs_goals.push_back(goals[i]);
    }
    std::vector<libMultiRobotPlanning::PlanResult<ECBSState, ECBSAction, int>>
        solution;
    auto ecbs_obstacles = std::unordered_set<Node>(graph.obstacles.begin(),
                                                   graph.obstacles.end());
    ECBSEnvironment ecbs_env(graph.x_size, graph.y_size, ecbs_obstacles,
                             ecbs_goals);
    libMultiRobotPlanning::ECBS<ECBSState, ECBSAction, int, ECBSConflict,
                                ECBSConstraints, ECBSEnvironment>
        ecbs_solver(ecbs_env, 1.5);
    // Perform search
    bool success = ecbs_solver.search(ecbs_starts, solution);
    assert(success);
    // Read and return the result.
    int makespan = 0;
    for (size_t i = 0; i < solution.size(); i++)
        if (makespan < solution[i].states.size())
            makespan = solution[i].states.size();
    auto paths = std::vector<std::vector<Node>>(makespan, goals);
    for (size_t a = 0; a < solution.size(); ++a)
        for (size_t t = 0; t < solution[a].states.size(); t++)
            paths[t][a] = Node(solution[a].states[t].first.x,
                               solution[a].states[t].first.y);
    return paths;
}

inline std::vector<Node> multigoal_find_goal(MultiGoalTask& task, Graph& graph,
                                             std::vector<Node>& starts,
                                             std::vector<size_t>& goal_stamps,
                                             size_t horizon, SUO* suo,
                                             size_t suo_rounds) {
    assert(starts.size() == goal_stamps.size());
    size_t num_robots = starts.size();
    // Step 0: identify goals
    auto goals = std::vector<Node>(num_robots);
    for (size_t i = 0; i < task.num_robots; i++) {
        if (goal_stamps[i] >= task.goals[i].size())
            goals[i] = task.goals[i].back();
        else
            goals[i] = task.goals[i][goal_stamps[i]];
    }
    // Step 1: get shortest paths
    auto initial_paths =
        std::vector<std::vector<Node>>(num_robots, std::vector<Node>());
    // Sort robots, with shorter path length in the front
    // TODO use longer and random path lengths as comparison
    auto robot_order = std::vector<size_t>(task.num_robots);
    auto path_lengths = std::vector<size_t>(task.num_robots);
    for (size_t i = 0; i < task.num_robots; i++) {
        robot_order[i] = i;
        path_lengths[i] = get_manhattan_distance(starts[i], goals[i]);
    }
    std::sort(robot_order.begin(), robot_order.end(),
              [&path_lengths](size_t a, size_t b) {
                  return path_lengths[a] < path_lengths[b];
              });
    // First, get an independent path for each robot
    auto path_planner =
        SingleRobotPathPlanner<OmniDirectionalRobot::State,
                               OmniDirectionalRobot::Action,
                               OmniDirectionalRobot::Environment>(graph);
    path_planner.set_suo(*suo);
    if (suo) suo->clear();
    // Multiple SUO rounds
    for (size_t r = 0; r < suo_rounds; r++) {
        for (size_t i = 0; i < task.num_robots; i++) {
            auto robot_index = robot_order[i];
            if (r > 0 && suo)
                suo->remove_path(initial_paths[robot_order[i]], true);
            initial_paths[robot_order[i]] = path_planner.reversed_search(
                starts[robot_order[i]], goals[robot_order[i]]);
            if (suo) suo->add_path(initial_paths[robot_order[i]], true);
        }
    }
    // Step 2: find priority
    auto robot_seq = std::vector<size_t>(num_robots);
    auto initial_path_sizes = std::vector<size_t>(num_robots);
    for (size_t i = 0; i < num_robots; i++) {
        robot_seq[i] = i;
        initial_path_sizes[i] = initial_paths[i].size();
    }
    std::sort(robot_seq.begin(), robot_seq.end(),
              [&initial_path_sizes](size_t a, size_t b) {
                  return initial_path_sizes[a] > initial_path_sizes[b];
              });
    // Step 5: find intermediate configs
    auto config = std::vector<Node>(num_robots);
    auto occupied_nodes = std::set<Node>();
    for (size_t i = 0; i < num_robots; i++) {
        auto robot_index = robot_seq[i];
        // Use BFS to find a vertex
        auto closed_set = std::set<Node>();
        int split_index = initial_paths[robot_index].size() - 1 - horizon;
        if (split_index < 0) split_index = 0;
        auto best_pt = initial_paths[robot_index][split_index];
        auto cmp = [&best_pt](Node left, Node right) {
            return get_manhattan_distance(left, best_pt) <
                   get_manhattan_distance(right, best_pt);
        };
        std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open_set(
            cmp);
        open_set.push(best_pt);
        while (!open_set.empty()) {
            auto n = open_set.top();
            open_set.pop();
            // Check if not occupied
            if (occupied_nodes.find(n) == occupied_nodes.end()) {
                config[robot_index] = n;
                occupied_nodes.insert(n);
                break;
            }
            if (closed_set.find(n) != closed_set.end()) continue;
            closed_set.insert(n);
            // Expand
            for (auto next_n : graph.adj_list.find(n)->second)
                if (closed_set.find(next_n) == closed_set.end())
                    open_set.push(next_n);
        }
    }
    return config;
}

inline size_t solve_multigoal(MultiGoalTask& task, Graph& graph,
                              size_t max_num_tasks, size_t horizon, SUO* suo,
                              size_t suo_rounds) {
    size_t finished = 0;
    size_t steps = 0;
    double max_time = 0;
    double total_time = 0;
    // Get starts and goals
    auto current_config = task.starts;
    auto goal_stamps = std::vector<size_t>(task.num_robots, 0);
    for (size_t i = 0; i < task.num_robots; i++)
        while (true) {
            if (goal_stamps[i] >= task.goals[i].size()) break;
            if (task.goals[i][goal_stamps[i]] != current_config[i]) break;
            goal_stamps[i] += 1;
            finished += 1;
        }
    // Starts to solve the problem
    while (finished < max_num_tasks) {
        auto start_time = std::chrono::high_resolution_clock::now();
        auto current_goal = multigoal_find_goal(
            task, graph, current_config, goal_stamps, horizon, suo, suo_rounds);
        auto paths = solve_ecbs(graph, current_config, current_goal);
        auto step_time = time_elapsed(start_time);
        // Robots execute paths
        current_config = paths[1];
        for (size_t i = 0; i < task.num_robots; i++)
            while (true) {
                if (goal_stamps[i] >= task.goals[i].size()) break;
                if (task.goals[i][goal_stamps[i]] != current_config[i]) break;
                goal_stamps[i] += 1;
                finished += 1;
            }
        total_time += step_time;
        if (step_time > max_time) max_time = step_time;
        steps += 1;
        std::cout << "Jobs finished: " << finished << "/" << max_num_tasks
                  << ", computation time per step: max " << max_time << "; avg "
                  << total_time / steps
                  << ", finished per step: " << double(finished) / steps
                  << '\r';
    }
    std::cout << "Jobs finished: " << finished << "/" << max_num_tasks
              << std::endl;
    return steps;
}
