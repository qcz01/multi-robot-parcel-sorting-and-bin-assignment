/**
 * @file suo.h
 * @author Shuai Han (shuai.han@rutgers.edu)
 * @brief Space Utilization Optimization (SUO) header
 * @version 0.1
 * @date 2021-01-09
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <unordered_map>
#include <vector>

#include "task.h"

/**
 * @brief Data structure to describe utilization, formatted as (prev_node,
 * current_node, time).
 *  When prev_node != current_node, we use it to denote an edge utilization.
 *  When prev_node != current_node, we use it to denote a vertex utilization.
 *
 */
typedef std::tuple<Node, Node, int> utilization_key;
namespace std {
template <>
struct hash<utilization_key> {
    size_t operator()(const utilization_key& p) const {
        size_t seed = 0;
        boost::hash_combine(seed, std::get<0>(p).x);
        boost::hash_combine(seed, std::get<0>(p).y);
        boost::hash_combine(seed, std::get<1>(p).y);
        boost::hash_combine(seed, std::get<1>(p).y);
        boost::hash_combine(seed, std::get<2>(p));
        return seed;
    }
};
}  // namespace std

/**
 * @brief Class for store and calculate space utilization
 *
 */
class SUO {
   public:
    SUO(){};
    SUO(double vertex_ratio_in, double edge_ratio_in, bool use_temporal_info_in,
        size_t time_expand_pre_in = 0, size_t time_expand_post_in = 0,
        bool use_as_cost_to_come_in = false)
        : vertex_ratio(vertex_ratio_in),
          edge_ratio(edge_ratio_in),
          use_temporal_info(use_temporal_info_in),
          time_expand_pre(time_expand_pre_in),
          time_expand_post(time_expand_post_in),
          use_as_cost_to_come(use_as_cost_to_come_in) {}
    SUO(const SUO& that) {
        vertex_ratio = that.vertex_ratio;
        edge_ratio = that.edge_ratio;
        use_temporal_info = that.use_temporal_info;
        time_expand_pre = that.time_expand_pre;
        time_expand_post = that.time_expand_post;
        occupancy = that.occupancy;
        occupancy_max = that.occupancy_max;
        use_as_cost_to_come = that.use_as_cost_to_come;
    };
    ~SUO(){};

    /**
     * @brief Record a path into the SUO database
     *
     * @param path: a list of node
     * @param reverse: whether to record in reverse mode: designed for DDM
     */
    inline void add_path(const std::vector<Node>& path, bool reverse = false) {
        change_occupancy_value(path, 1, reverse);
    }
    /**
     * @brief Remove a path from the SUO database
     *
     * @param path: a list of node
     * @param reverse: whether to record in reverse mode: designed for DDM
     */
    inline void remove_path(const std::vector<Node>& path,
                            bool reverse = false) {
        change_occupancy_value(path, -1, reverse);
    }

    /**
     * @brief Get the SUO heuristic value
     *
     * @param before: prev_node
     * @param after: current_node
     * @param time:
     * @return double
     */
    inline double get_heuristic_value(Node before, Node after,
                                      size_t time) const {
        int t = use_temporal_info ? time : -1;
        // Check vertex occupancy value
        auto it_v = occupancy.find(utilization_key(after, after, t));
        double vertex_occupancy = (it_v == occupancy.end() ? 0 : it_v->second);
        // Check edge occupancy value
        auto it_e =
            // occupancy.find(utilization_key(after, before, t));
            occupancy.find(utilization_key(after, before, t));
        double edge_occupancy = (it_e == occupancy.end() ? 0 : it_e->second);
        // Calculate and output
        return ((vertex_occupancy / occupancy_max) * vertex_ratio +
                (edge_occupancy / occupancy_max) * edge_ratio) *
               0.5;
    }

    /**
     * @brief Remove all occupancy data. Useful when doing a complete
     * replanning.
     *
     */
    inline void clear() {
        occupancy.clear();
        occupancy_max = 1;
    }

    // Setup variables
    double vertex_ratio = 1;         // Ratio factor on node occupancy
    double edge_ratio = 1;           // Ratio factor on edge occupancy
    bool use_temporal_info = false;  // Whether to use time information
    size_t time_expand_pre =
        0;  // When using time information, how many time steps
            // to expand before a robot reaches a Node
    size_t time_expand_post =
        0;  // When using time information, how many time steps
            // to expand after a robot reaches a Node
    bool use_as_cost_to_come = false;

   private:
    std::unordered_map<utilization_key, double, std::hash<utilization_key>>
        occupancy =
            std::unordered_map<utilization_key, double,
                               std::hash<utilization_key>>();  // Database
    double occupancy_max =
        1;  // Keep track of the max occupancy; used for normalization

    /**
     * @brief Helper function to add/remove occupancy data
     *
     * @param path_in:
     * @param amount: how much to change
     * @param reverse: whether the path is in reverse
     */
    inline void change_occupancy_value(const std::vector<Node>& path,
                                       double amount, bool reverse) {
        if (!reverse) {
            for (int i = 0; i < path.size(); i++) {
                if (use_temporal_info) {
                    for (int t =
                             (i > time_expand_pre ? i - time_expand_pre : 0);
                         t <= i + time_expand_post; t++) {
                        // Add vertex info
                        change_occupancy_value(path[i], path[i], t, amount);
                        // Add edge info
                        if (i > 0)
                            change_occupancy_value(path[i - 1], path[i], t,
                                                   amount);
                    }
                } else {
                    change_occupancy_value(path[i], path[i], -1, amount);
                    if (i > 0)
                        change_occupancy_value(path[i - 1], path[i], -1,
                                               amount);
                }
            }
        } else {
            for (int i = path.size() - 1; i >= 0; i--) {
                if (use_temporal_info) {
                    for (int t = (i < path.size() - 1 - time_expand_pre
                                      ? i + time_expand_pre
                                      : path.size() - 1);
                         t >= i - time_expand_post; t--) {
                        change_occupancy_value(path[i], path[i],
                                               path.size() - 1 - t, amount);
                        if (i < path.size() - 1)
                            change_occupancy_value(path[i + 1], path[i],
                                                   path.size() - 1 - t, amount);
                    }
                } else {
                    change_occupancy_value(path[i], path[i], -1, amount);
                    if (i < path.size() - 1)
                        change_occupancy_value(path[i + 1], path[i], -1,
                                               amount);
                }
            }
        }
    }

    /**
     * @brief Lower helper function for changing the database
     *
     * @param before:
     * @param after:
     * @param time:
     * @param amount:
     */
    inline void change_occupancy_value(const Node& before, const Node& after,
                                       int time, double amount) {
        auto key = utilization_key(before, after, time);
        auto it = occupancy.find(key);
        if (it != occupancy.end()) {
            it->second += amount;
            if (it->second > occupancy_max) occupancy_max = it->second;
        } else {
            if (amount >= 0)
                occupancy.insert(make_pair(key, amount));
            else
                throw "Occupancy amount less than 0";
        }
    }
};
