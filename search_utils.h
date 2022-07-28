/**
 * @file search_utils.h
 * @author Shuai Han (shuai.han@rutgers.edu)
 * @brief This function calls the customized A* for single robot path planning
 * @version 0.1
 * @date 2021-01-09
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <fstream>
#include <iostream>
#include <chrono>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include "custom_astar.h"
#include "suo.h"
#include "task.h"

#include <libMultiRobotPlanning/a_star_epsilon.hpp>


inline double time_elapsed_debug(
    std::chrono::time_point<std::chrono::high_resolution_clock>& start_time) {
    return double(static_cast<long long int>(
               std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now() - start_time)
                   .count())) /
           1000000;
}

/**
 * @brief The namespace for a robot model.
 * TODO after we have multiple models, the namespaces should be linked to the
 * tasks as template.
 *
 */
namespace OmniDirectionalRobot {

using libMultiRobotPlanning::AStar;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::SinglePlanResult;

/**
 * @brief Robot state. For omnidirectional robot it's just the location
 *
 */
struct State {
    State() {}
    State(int x, int y) : n(x, y) {}
    State(int x, int y, int t) : n(x, y), t(t) {}
    State(const Node& n_in) : n(n_in) {}

    State(const State&) = default;
    State(State&&) = default;
    State& operator=(const State&) = default;
    State& operator=(State&&) = default;

    bool operator==(const State& other) const { return n == other.n; }
    bool operator!=(const State& other) const { return !(*this == other); }

    friend std::ostream& operator<<(std::ostream& os, const State& s) {
        return os << s.n;
    }

    Node n;
    int t = 0;  // Time reached this state, used for checking the SUO heuristic
};

/**
 * @brief Transition actions specified by the robot model
 *
 */
enum class Action {
    Up,
    Down,
    Left,
    Right,
    Wait,
};
inline std::ostream& operator<<(std::ostream& os, const Action& a) {
    switch (a) {
        case Action::Up:
            os << "Up";
            break;
        case Action::Down:
            os << "Down";
            break;
        case Action::Left:
            os << "Left";
            break;
        case Action::Right:
            os << "Right";
            break;
        case Action::Wait:
            os << "Wait";
            break;
    }
    return os;
}

/**
 * @brief Env customized according to the custom A* class
 *
 */
class Environment {
   public:
    using Paths=std::vector<std::vector<Node>>;
    Environment(const Graph& g_in) : g(g_in) {}
    Environment(const Graph& g_in, State goal_in)
        : g(g_in), m_goal(std::move(goal_in)) {}

    /**
     * @brief Heuristic function called by A*
     *
     * @param s_prev: previous state
     * @param s: next state
     * @return double
     */
    double admissibleHeuristic(const State& s_prev, const State& s) {
        // TODO optionally replace manhattan distance with actual shortest path
        // length
        if (suo)
            if (!suo->use_as_cost_to_come)
                return get_manhattan_distance(s.n, m_goal.n) +
                       suo->get_heuristic_value(s_prev.n, s.n, s.t);
        return get_manhattan_distance(s.n, m_goal.n);
    }

    double true_distance(const State &s){
        Node start=s.n;
        Node goal=m_goal.n;
    }


    double admissibleHeuristic(const State &s){
        return get_manhattan_distance(s.n,m_goal.n); 
    }

    /**
     * @brief Check whether the state is a goal state
     *
     * @param s
     * @return true
     * @return false
     */
    bool isSolution(const State& s) { return s == m_goal; }

    /**
     * @brief Get the neighboring states
     *
     * @param s_in
     * @param neighbors: place to output neighbors
     */
    void getNeighbors(const State& s_in,
                      std::vector<Neighbor<State, Action, double>>& neighbors) {
        neighbors.clear();
        static std::vector<Action> actions({Action::Up, Action::Down,
                                            Action::Left, Action::Right,
                                            Action::Wait});
        for (auto a : actions) {
            State s = takeAction(s_in, a);
            if (transitionValid(s_in, s)) {
                neighbors.emplace_back(
                    Neighbor<State, Action, double>(s, a, 1));
            }
        }
    }

    /**
     * @brief Check if a transition is valid
     *
     * @param s_prev: previous state
     * @param s_after: next state
     * @return true
     * @return false
     */
    bool transitionValid(const State& s_prev, const State& s_after) {
        Node sn=s_prev.n;
        Node gn=s_after.n;
        auto adj=g.adj_list;
        auto it_s=adj[sn].begin();
        auto it_g=adj[sn].end();

        return g.has_node(s_after.n) && (!g.is_blocked(s_after.n)) && (std::find(it_s,it_g,gn)!=it_g);
    }

    /**
     * @brief Get the next state after applying an action to the current state
     *
     * @param s
     * @param a
     * @return State
     */
    inline State takeAction(const State& s, Action a) const {
        switch (a) {
            case Action::Up:
                return State(s.n.x, s.n.y + 1, s.t + 1);
            case Action::Down:
                return State(s.n.x, s.n.y - 1, s.t + 1);
            case Action::Left:
                return State(s.n.x - 1, s.n.y, s.t + 1);
            case Action::Right:
                return State(s.n.x + 1, s.n.y, s.t + 1);
            case Action::Wait:
                return State(s.n.x, s.n.y, s.t + 1);
        };
    }

    void setGoal(const Node& goal) { m_goal = State(goal); }
    void setGoal(const State& goal) { m_goal = goal; }

    void onExpandNode(const State& /*s*/, double /*fScore*/,
                      double /*gScore*/) {}

    void onDiscover(const State& /*s*/, double /*fScore*/, double /*gScore*/) {}

    Paths path_table;

    int agent_id;


    double focalStateHeuristic(const State &s, double gScore)
    {
        int num_conflicts=0;
        for(int i=0;i<path_table.size();i++){
            if(i==agent_id) continue;
            int li=path_table[i].size();
            if(s.t>=li) continue;
            if(path_table[i][li-1-s.t]==s.n) num_conflicts++;
        }
        return num_conflicts;
    }

    double focalTransitionHeuristic(const State &s1, const State &s2,
                                double gScoreS1, double gScoreS2)
    {
        int num_conflicts=0;
        // for(int i=0;i<path_table.size();i++){
        //     if(i==agent_id) continue;
        //     if(s2.t>=path_table[i].size()) continue;
        //     if(path_table[i][s2.t]==s1.n and path_table[i][s1.t]==s2.n) num_conflicts++;
        // }
        return num_conflicts;
    }

    const Graph& g;
    State m_goal;  // Single A* goal
    SUO* suo = nullptr;
};
}  // namespace OmniDirectionalRobot

/**
 * @brief Hashing function for OmniDirectionalRobot::State
 *
 */
namespace std {
template <>
struct hash<OmniDirectionalRobot::State> {
    size_t operator()(const OmniDirectionalRobot::State& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.n.x);
        boost::hash_combine(seed, s.n.y);
        return seed;
    }
};
}  // namespace std

/**
 * @brief Packed function for single robot path planning
 * TODO will be updated
 *
 * @tparam State:
 * @tparam Action:
 * @tparam Environment:
 */
template <typename State, typename Action, typename Environment>
class SingleRobotPathPlanner {
   public:
   using Paths=std::vector<std::vector<Node>>;
    SingleRobotPathPlanner(const Graph& g) {
        env = new Environment(g);
        searcher = new libMultiRobotPlanning::AStar<State, Action, double,
                                                    Environment>(*env);
        focal_searcher=new libMultiRobotPlanning::AStarEpsilon<State,Action,double,Environment>(*env,1.2);
    }
    ~SingleRobotPathPlanner() {
        delete env;
        delete searcher;
    }

    /**
     * @brief Search for a path given the start and goal node (omnidirectional
     * robot specific)
     *
     * @param start: graph node
     * @param goal: graph node
     * @return libMultiRobotPlanning::PlanResult<State, Action, double>
     */
    inline libMultiRobotPlanning::SinglePlanResult<State, Action, double>
    search(Node start, Node goal) {
        return search(State(start), State(goal));
    }


    libMultiRobotPlanning::PlanResult<State, Action, double> search_focal(State start,State goal,int agent_id,Paths&path_table){
        env->setGoal(goal);
        env->agent_id=agent_id;
        env->path_table=path_table;
        libMultiRobotPlanning::PlanResult<State, Action, double> solution;
        bool search_success=focal_searcher->search(start,solution);
        return solution;

    }

    

    /**
     * @brief Search for a path given the start and goal state (appllies to any
     * robot model)
     *
     * @param start: start state
     * @param goal: goal state
     * @return libMultiRobotPlanning::PlanResult<State, Action, double>
     */
    inline libMultiRobotPlanning::SinglePlanResult<State, Action, double>
    search(State start, State goal) {
        env->setGoal(goal);
        libMultiRobotPlanning::SinglePlanResult<State, Action, double> solution;
        bool search_success = searcher->search(start, solution);
        assert(search_success);
        return solution;
    }

    /**
     * @brief Set the suo processor
     * TODO revisit and check whether there are better ways to do this
     *
     * @param suo:
     */
    inline void set_suo(SUO& suo) { env->suo = &suo; }

    /**
     * @brief Given the start and goal node, search in reverse and return nodes
     * on the path (omnidirectional robot specific, designed for DDM)
     *
     * @param start:
     * @param goal:
     * @return std::vector<Node>
     */
    inline std::vector<Node> reversed_search(Node start, Node goal) {
        // auto start_time=std::chrono::high_resolution_clock::now();
        auto result = search(start, goal);
        // double time_cost=time_elapsed_debug(start_time);
        // std::cout<<"elpased "<<time_cost<<"   "<<start<<"   "<<goal<<std::endl;
        auto path = std::vector<Node>();
        path.reserve(result.states.size());
        for (int i = result.states.size() - 1; i >= 0; i--)
            path.push_back(result.states[i].n);
        return path;
    }



    inline std::vector<Node> reversed_search_focal(Node start, Node goal,int agent_id,Paths &path_table) {
        auto result = search_focal(State(start), State(goal),agent_id,path_table);
        auto path = std::vector<Node>();
        path.reserve(result.states.size());
        for (int i = result.states.size() - 1; i >= 0; i--)
            path.push_back(result.states[i].first.n);
        return path;
    }

    libMultiRobotPlanning::AStar<State, Action, double, Environment>* searcher;
    libMultiRobotPlanning::AStarEpsilon<State,Action,double,Environment> *focal_searcher;
    Environment* env;
};