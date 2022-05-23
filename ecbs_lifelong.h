#pragma once

#include <fstream>
#include <iostream>
#include <numeric>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include "ecbs_customized.h"
#include "suo.h"
#include "task.h"
#include "utils.h"

/**
 * @brief A search state for a single robot in ECBS.
 * TODO: merge this with other states.
 *
 */
struct ECBSState {
    ECBSState(Node n, int time, size_t label)
        : n(n), time(time), label(label) {}

    bool operator==(const ECBSState& s) const {
        return time == s.time && n == s.n && label == s.label;
    }
    bool equalNode(const ECBSState& s) const { return n == s.n; }
    inline friend std::ostream& operator<<(std::ostream& os,
                                           const ECBSState& s) {
        return os << s.time << ": (" << s.n.x << "," << s.n.y << ")" << s.label;
    }
    int time;
    Node n;
    size_t label;
};
namespace std {
template <>
struct hash<ECBSState> {
    size_t operator()(const ECBSState& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.time);
        boost::hash_combine(seed, s.n.x);
        boost::hash_combine(seed, s.n.y);
        boost::hash_combine(seed, s.label);
        return seed;
    }
};
}  // namespace std
/**
 * @brief TODO: merge this with other actions.
 *
 */
enum class ECBSAction {
    Up,
    Down,
    Left,
    Right,
    Wait,
};
inline std::ostream& operator<<(std::ostream& os, const ECBSAction& a) {
    switch (a) {
        case ECBSAction::Up:
            os << "Up";
            break;
        case ECBSAction::Down:
            os << "Down";
            break;
        case ECBSAction::Left:
            os << "Left";
            break;
        case ECBSAction::Right:
            os << "Right";
            break;
        case ECBSAction::Wait:
            os << "Wait";
            break;
    }
    return os;
}

/**
 * @brief Describes a conflict between two robots at a vertex (or an edge) at a
 * time step
 *
 */
struct ECBSConflict {
    enum Type {
        Vertex,
        Edge,
    };

    int time;       // Conflict time
    size_t agent1;  // Conflicting robot index
    size_t agent2;  // Conflicting robot index
    Type type;      // Conflict type

    Node n1;  // Vertex conflict: the conflict node; edge conflict: the first
              // conflict node
    Node n2;  // Vertex conflict: NA; edge conflict: the second conflict node

    inline friend std::ostream& operator<<(std::ostream& os,
                                           const ECBSConflict& c) {
        switch (c.type) {
            case Vertex:
                return os << c.time << ": Vertex(" << c.n1 << ")";
            case Edge:
                return os << c.time << ": Edge(" << c.n1 << "," << c.n2 << ")";
        }
        return os;
    }
};

/**
 * @brief Cannot go to a vertex at a time step
 *
 */
struct ECBSVertexConstraint {
    ECBSVertexConstraint(int time, Node n) : time(time), n(n) {}
    int time;
    Node n;

    bool operator<(const ECBSVertexConstraint& other) const {
        return std::tie(time, n.x, n.y) <
               std::tie(other.time, other.n.x, other.n.y);
    }
    bool operator==(const ECBSVertexConstraint& other) const {
        return std::tie(time, n.x, n.y) ==
               std::tie(other.time, other.n.x, other.n.y);
    }
    inline friend std::ostream& operator<<(std::ostream& os,
                                           const ECBSVertexConstraint& c) {
        return os << "VC(" << c.time << "," << c.n.x << "," << c.n.y << ")";
    }
};
namespace std {
template <>
struct hash<ECBSVertexConstraint> {
    size_t operator()(const ECBSVertexConstraint& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.time);
        boost::hash_combine(seed, s.n.x);
        boost::hash_combine(seed, s.n.y);
        return seed;
    }
};
}  // namespace std

/**
 * @brief Cannot go through an edge at a time step
 *
 */
struct ECBSEdgeConstraint {
    ECBSEdgeConstraint(int time, Node n1, Node n2)
        : time(time), n1(n1), n2(n2) {}
    int time;
    Node n1;
    Node n2;

    bool operator<(const ECBSEdgeConstraint& other) const {
        return std::tie(time, n1.x, n1.y, n2.x, n2.y) <
               std::tie(other.time, other.n1.x, other.n1.y, other.n2.x,
                        other.n2.y);
    }
    bool operator==(const ECBSEdgeConstraint& other) const {
        return std::tie(time, n1.x, n1.y, n2.x, n2.y) ==
               std::tie(other.time, other.n1.x, other.n1.y, other.n2.x,
                        other.n2.y);
    }
    inline friend std::ostream& operator<<(std::ostream& os,
                                           const ECBSEdgeConstraint& c) {
        return os << "EC(" << c.time << "," << c.n1 << "," << c.n2 << ")";
    }
};
namespace std {
template <>
struct hash<ECBSEdgeConstraint> {
    size_t operator()(const ECBSEdgeConstraint& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.time);
        boost::hash_combine(seed, s.n1.x);
        boost::hash_combine(seed, s.n1.y);
        boost::hash_combine(seed, s.n2.x);
        boost::hash_combine(seed, s.n2.y);
        return seed;
    }
};
}  // namespace std

/**
 * @brief A collection of constraints. Can be sent to A* algorithm to avoid
 * collisions
 *
 */
struct ECBSConstraints {
    std::unordered_set<ECBSVertexConstraint> vertexConstraints;
    std::unordered_set<ECBSEdgeConstraint> edgeConstraints;
    /**
     * @brief Conbine two sets of constraints (in place)
     *
     * @param other:
     */
    void add(const ECBSConstraints& other) {
        vertexConstraints.insert(other.vertexConstraints.begin(),
                                 other.vertexConstraints.end());
        edgeConstraints.insert(other.edgeConstraints.begin(),
                               other.edgeConstraints.end());
    }
    bool overlap(const ECBSConstraints& other) const { return false; }
    inline friend std::ostream& operator<<(std::ostream& os,
                                           const ECBSConstraints& c) {
        for (const auto& vc : c.vertexConstraints) {
            os << vc << std::endl;
        }
        for (const auto& ec : c.edgeConstraints) {
            os << ec << std::endl;
        }
        return os;
    }
};

/**
 * @brief Searching environment
 *
 */
class ECBSEnvironment {
   public:
    /**
     * @brief Construct a new ECBSEnvironment object
     *
     * @param graph: graph
     * @param goals: multi goals
     * @param horizon: horizon for resolving conflicts
     */
    ECBSEnvironment(const Graph& graph,
                    const std::vector<std::vector<Node>>& goals, int horizon)
        : m_graph(&graph),
          m_goals(goals),
          m_horizon(horizon),
          m_agentIdx(0),
          m_constraints(nullptr),
          m_highLevelExpanded(0),
          m_lowLevelExpanded(0),
          m_lastGoalConstraint(-1) {}
    ECBSEnvironment(const ECBSEnvironment&) = delete;
    ECBSEnvironment& operator=(const ECBSEnvironment&) = delete;
    /**
     * @brief Set the Low Level search focus
     *
     * @param agentIdx: robot index
     * @param constraints: robot constraints
     */
    void setLowLevelContext(size_t agentIdx,
                            const ECBSConstraints* constraints) {
        assert(constraints);
        m_agentIdx = agentIdx;
        m_constraints = constraints;
        // Update to keep search active
        m_lastGoalConstraint = -1;
        for (const auto& vc : constraints->vertexConstraints) {
            if (vc.n == m_goals[m_agentIdx].back()) {
                m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
            }
        }
    }
    /**
     * @brief Get admissible heuristic, with distance to the next goal, and all
     * the residual distance to the remaining goals
     *
     * @param s:
     * @return double
     */
    double admissibleHeuristic(const ECBSState& s) {
        double ret_val =
            get_manhattan_distance(s.n, m_goals[m_agentIdx][s.label]);
        for (auto i = s.label + 1; i < m_goals[m_agentIdx].size(); i++)
            ret_val += get_manhattan_distance(m_goals[m_agentIdx][i - 1],
                                              m_goals[m_agentIdx][i]);
        return ret_val;
    }
    /**
     * @brief Check the total number of vertex conflicts between the current
     * robot's path and all other paths. Used for low level search
     *
     * @param s: state
     * @param gScore: not used
     * @param solution: current paths
     * @return double
     */
    double focalStateHeuristic(
        const ECBSState& s, double gScore,
        const std::vector<
            libMultiRobotPlanning::PlanResult<ECBSState, ECBSAction, double>>&
            solution) {
        int numConflicts = 0;
        for (size_t i = 0; i < solution.size(); i++) {
            if (i != m_agentIdx && !solution[i].states.empty()) {
                ECBSState state2 = getState(i, solution, s.time);
                if (s.equalNode(state2)) numConflicts++;
            }
        }
        // return numConflicts + n_suo->get_heuristic_value(s.n, s.n, 0);
        return numConflicts;
    }
    /**
     * @brief Check the total number of edge conflicts between the current
     * robot's path and all other paths. Used for low level search.
     *
     * @param s1a: transition prev
     * @param s1b: transition next
     * @param gScoreS1a: not used
     * @param gScoreS1b: not used
     * @param solution: current paths
     * @return int
     */
    int focalTransitionHeuristic(
        const ECBSState& s1a, const ECBSState& s1b, double gScoreS1a,
        double gScoreS1b,
        const std::vector<
            libMultiRobotPlanning::PlanResult<ECBSState, ECBSAction, double>>&
            solution) {
        int numConflicts = 0;
        for (size_t i = 0; i < solution.size(); ++i) {
            if (i != m_agentIdx && !solution[i].states.empty()) {
                ECBSState s2a = getState(i, solution, s1a.time);
                ECBSState s2b = getState(i, solution, s1b.time);
                if (s1a.equalNode(s2b) && s1b.equalNode(s2a)) ++numConflicts;
            }
        }
        return numConflicts;
    }
    /**
     * @brief Count the total number of conflicts in a high level node. Horizon
     * considered
     *
     * @param solution: current paths
     * @return int
     */
    int focalHeuristic(const std::vector<libMultiRobotPlanning::PlanResult<
                           ECBSState, ECBSAction, double>>& solution) {
        int numConflicts = 0;
        // get makespan
        int makespan = 0;
        for (const auto& sol : solution)
            makespan = std::max<int>(makespan, sol.states.size() - 1);
        makespan = std::min(makespan, m_horizon);
        for (int t = 0; t <= makespan; t++) {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i) {
                ECBSState state1 = getState(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    ECBSState state2 = getState(j, solution, t);
                    if (state1.equalNode(state2)) ++numConflicts;
                }
            }
            // drive-drive edge (swap)
            if (t < makespan) {
                for (size_t i = 0; i < solution.size(); ++i) {
                    ECBSState state1a = getState(i, solution, t);
                    ECBSState state1b = getState(i, solution, t + 1);
                    for (size_t j = i + 1; j < solution.size(); ++j) {
                        ECBSState state2a = getState(j, solution, t);
                        ECBSState state2b = getState(j, solution, t + 1);
                        if (state1a.equalNode(state2b) &&
                            state1b.equalNode(state2a))
                            ++numConflicts;
                    }
                }
            }
        }
        return numConflicts;
    }
    /**
     * @brief Check if search finished. Condition: reached the last goal
     *
     * @param s: current state
     * @return true
     * @return false
     */
    bool isSolution(const ECBSState& s) {
        return s.label == m_goals[m_agentIdx].size() - 1 &&
               s.n == m_goals[m_agentIdx].back() &&
               s.time > m_lastGoalConstraint;
    }
    /**
     * @brief Get the next state after applying an action to the current state
     *
     * @param s
     * @param a
     * @return State
     */
    inline ECBSState takeAction(const ECBSState& s, ECBSAction a) const {
        switch (a) {
            case ECBSAction::Up:
                return ECBSState(Node(s.n.x, s.n.y + 1), s.time + 1, s.label);
            case ECBSAction::Down:
                return ECBSState(Node(s.n.x, s.n.y - 1), s.time + 1, s.label);
            case ECBSAction::Left:
                return ECBSState(Node(s.n.x - 1, s.n.y), s.time + 1, s.label);
            case ECBSAction::Right:
                return ECBSState(Node(s.n.x + 1, s.n.y), s.time + 1, s.label);
            case ECBSAction::Wait:
                return ECBSState(Node(s.n.x, s.n.y), s.time + 1, s.label);
        };
    }
    /**
     * @brief Check if a low level transition is valid: next node in graph, not
     * obstacle; not avoiding any constraint
     *
     * @param s1:
     * @param s2:
     * @return true
     * @return false
     */
    bool transitionValid(const ECBSState& s1, const ECBSState& s2) {
        assert(m_constraints);
        const auto& vcon = m_constraints->vertexConstraints;
        const auto& econ = m_constraints->edgeConstraints;
        return m_graph->has_node(s2.n) && (!m_graph->is_blocked(s2.n)) &&
               vcon.find(ECBSVertexConstraint(s2.time, s2.n)) == vcon.end() &&
               econ.find(ECBSEdgeConstraint(s1.time, s1.n, s2.n)) == econ.end();
    }
    /**
     * @brief Get the neighbors during low level search
     *
     * @param s_in:
     * @param neighbors:
     */
    void getNeighbors(const ECBSState& s_in,
                      std::vector<libMultiRobotPlanning::Neighbor<
                          ECBSState, ECBSAction, double>>& neighbors) {
        neighbors.clear();
        static std::vector<ECBSAction> actions(
            {ECBSAction::Up, ECBSAction::Down, ECBSAction::Left,
             ECBSAction::Right, ECBSAction::Wait});
        for (auto a : actions) {
            ECBSState s = takeAction(s_in, a);
            if (transitionValid(s_in, s)) {
                // Check label
                if (s.n == m_goals[m_agentIdx][s.label] &&
                    s.label < m_goals[m_agentIdx].size() - 1)
                    s.label += 1;
                neighbors.emplace_back(
                    libMultiRobotPlanning::Neighbor<ECBSState, ECBSAction,
                                                    double>(s, a, 1));
            }
        }
    }
    /**
     * @brief Get the very first conflicts
     *
     * @param solution:
     * @param result:
     * @return true
     * @return false
     */
    bool getFirstConflict(const std::vector<libMultiRobotPlanning::PlanResult<
                              ECBSState, ECBSAction, double>>& solution,
                          ECBSConflict& result) {
        // get makespan
        int makespan = 0;
        for (const auto& sol : solution)
            makespan = std::max<int>(makespan, sol.states.size() - 1);
        makespan = std::min(makespan, m_horizon);
        for (int t = 0; t <= makespan; t++) {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i) {
                ECBSState state1 = getState(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    ECBSState state2 = getState(j, solution, t);
                    if (state1.equalNode(state2)) {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = ECBSConflict::Vertex;
                        result.n1 = state1.n;
                        return true;
                    }
                }
            }
            // drive-drive edge (swap)
            if (t < makespan) {
                for (size_t i = 0; i < solution.size(); ++i) {
                    ECBSState state1a = getState(i, solution, t);
                    ECBSState state1b = getState(i, solution, t + 1);
                    for (size_t j = i + 1; j < solution.size(); ++j) {
                        ECBSState state2a = getState(j, solution, t);
                        ECBSState state2b = getState(j, solution, t + 1);
                        if (state1a.equalNode(state2b) &&
                            state1b.equalNode(state2a)) {
                            result.time = t;
                            result.agent1 = i;
                            result.agent2 = j;
                            result.type = ECBSConflict::Edge;
                            result.n1 = state1a.n;
                            result.n2 = state1b.n;
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }
    /**
     * @brief Create a Constraints From Conflict object
     *
     * @param conflict:
     * @param constraints:
     */
    void createConstraintsFromConflict(
        const ECBSConflict& conflict,
        std::map<size_t, ECBSConstraints>& constraints) {
        if (conflict.type == ECBSConflict::Vertex) {
            ECBSConstraints c1;
            c1.vertexConstraints.emplace(
                ECBSVertexConstraint(conflict.time, conflict.n1));
            constraints[conflict.agent1] = c1;
            constraints[conflict.agent2] = c1;
        } else if (conflict.type == ECBSConflict::Edge) {
            ECBSConstraints c1;
            c1.edgeConstraints.emplace(
                ECBSEdgeConstraint(conflict.time, conflict.n1, conflict.n2));
            constraints[conflict.agent1] = c1;
            ECBSConstraints c2;
            c2.edgeConstraints.emplace(
                ECBSEdgeConstraint(conflict.time, conflict.n2, conflict.n1));
            constraints[conflict.agent2] = c2;
        }
    }
    void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }
    void onExpandLowLevelNode(const ECBSState& /*s*/, int /*fScore*/,
                              int /*gScore*/) {
        m_lowLevelExpanded++;
    }
    int highLevelExpanded() { return m_highLevelExpanded; }
    int lowLevelExpanded() const { return m_lowLevelExpanded; }

   private:
    /**
     * @brief Extract a state from a robot's path
     *
     * @param agentIdx:
     * @param solution:
     * @param t:
     * @return ECBSState
     */
    ECBSState getState(size_t agentIdx,
                       const std::vector<libMultiRobotPlanning::PlanResult<
                           ECBSState, ECBSAction, double>>& solution,
                       size_t t) {
        assert(agentIdx < solution.size());
        if (t < solution[agentIdx].states.size())
            return solution[agentIdx].states[t].first;
        assert(!solution[agentIdx].states.empty());
        return solution[agentIdx].states.back().first;
    }

   public:
    /**
     * @brief Get a robot's path
     *
     * @param agentIdx:
     * @param solution:
     * @return std::vector<Node>
     */
    std::vector<Node> getSinglePath(
        size_t agentIdx, const std::vector<libMultiRobotPlanning::PlanResult<
                             ECBSState, ECBSAction, double>>& solution) {
        assert(agentIdx < solution.size());
        auto ret_val = std::vector<Node>();
        ret_val.reserve(solution[agentIdx].states.size());
        for (auto& s : solution[agentIdx].states) ret_val.push_back(s.first.n);
        return ret_val;
    }

   private:
    const Graph* m_graph;                          // Graph
    const std::vector<std::vector<Node>> m_goals;  // List of goals
    int m_horizon;                                 // The horizon for searching
    size_t m_agentIdx;                             // Low-level search focus
    const ECBSConstraints* m_constraints;          // Low-level constraint focus
    int m_highLevelExpanded;                       // Info variable
    int m_lowLevelExpanded;                        // Info variable
    int m_lastGoalConstraint;  // To keep search active when the other robot
                               // might collide into the current robot. Has no
                               // effect when horizon is less than the shortest
                               // distance to all goals
};

class ECBSSolver {
   public:
    ECBSSolver(){};
    ECBSSolver(const ECBSSolver& that)
        : suo(that.suo),
          use_suo(that.use_suo),
          default_horizon(that.default_horizon),
          adaptive_horizon(that.adaptive_horizon){};
    /**
     * @brief Solve a multi step problem by one horizon
     *
     * @param task:
     * @param graph:
     * @param config: current robot locations
     * @param labels: current problem solving status
     * @param horizon: window size
     * @return std::vector<std::vector<Node>>
     */
    std::vector<std::vector<Node>> solve_step(
        const MultiGoalTask& task, const Graph& graph, std::vector<Node> config,
        std::vector<size_t> labels,
        int horizon = std::numeric_limits<int>::max(), double w = 1.5) {
        suo->clear();
        // Setup searching environment
        auto robot_order = std::vector<size_t>(task.num_robots);
        auto path_lengths = std::vector<size_t>(task.num_robots);
        auto ecbs_starts = std::vector<ECBSState>();
        auto ecbs_goals = std::vector<std::vector<Node>>();
        for (size_t i = 0; i < task.num_robots; i++) {
            robot_order[i] = i;
            // Get the start node, which is the current config
            ecbs_starts.push_back(ECBSState(config[i], 0, 0));
            // Get the next goal node
            ecbs_goals.push_back(std::vector<Node>({task.goals[i][labels[i]]}));
            int total_distance =
                get_manhattan_distance(config[i], task.goals[i][labels[i]]);
            size_t label_increment = 0;
            // Add goals until lb reaches horizon
            while (total_distance < horizon) {
                label_increment++;
                // If goals are exhausted, return
                if (labels[i] + label_increment == task.goals[i].size()) break;
                ecbs_goals[i].push_back(
                    task.goals[i][labels[i] + label_increment]);
                path_lengths[i] = get_manhattan_distance(
                    task.goals[i][labels[i] + label_increment - 1],
                    task.goals[i][labels[i] + label_increment]);
                total_distance += path_lengths[i];
            }
        }
        if (use_suo) {
            // Setup priority. Note we change priority to favor robots closer to
            // the goals
            std::sort(robot_order.begin(), robot_order.end(),
                      [&path_lengths](size_t a, size_t b) {
                          return path_lengths[a] > path_lengths[b];
                      });
            // Setup single robot searching
            auto path_planner =
                SingleRobotPathPlanner<OmniDirectionalRobot::State,
                                       OmniDirectionalRobot::Action,
                                       OmniDirectionalRobot::Environment>(
                    graph);
            path_planner.set_suo(*suo);
            suo->clear();
            auto initial_paths = std::vector<std::vector<Node>>(
                task.num_robots, std::vector<Node>());
            // Get paths
            for (size_t i = 0; i < task.num_robots; i++) {
                initial_paths[i] =
                    path_planner.reversed_search(config[i], ecbs_goals[i][0]);
                std::reverse(initial_paths[i].begin(), initial_paths[i].end());
                suo->add_path(initial_paths[i], false);
                for (size_t t = 0; t < ecbs_goals[i].size() - 1; t++) {
                    auto path = path_planner.reversed_search(
                        ecbs_goals[i][t], ecbs_goals[i][t + 1]);
                    path.pop_back();
                    std::reverse(path.begin(), path.end());
                    suo->add_path(path, false);
                    initial_paths[i].insert(initial_paths[i].end(),
                                            path.begin(), path.end());
                }
                if (initial_paths[i].size() > horizon + 2)
                    ecbs_goals[i].back() = initial_paths[i][horizon + 2];
            }
        }
        std::vector<
            libMultiRobotPlanning::PlanResult<ECBSState, ECBSAction, double>>
            solution;
        ECBSEnvironment ecbs_env(graph, ecbs_goals, horizon);
        libMultiRobotPlanning::ECBS<ECBSState, ECBSAction, double, ECBSConflict,
                                    ECBSConstraints, ECBSEnvironment>
            ecbs_solver(ecbs_env, w);
        // Perform search
        bool success = ecbs_solver.search(ecbs_starts, solution);
        assert(success);
        // Read and return the result.
        int makespan = 0;
        for (size_t i = 0; i < solution.size(); i++)
            if (makespan < solution[i].states.size() - 1)
                makespan = solution[i].states.size() - 1;
        auto paths = std::vector<std::vector<Node>>(
            makespan + 1, std::vector<Node>(task.num_robots, Node(-1, -1)));
        for (size_t a = 0; a < solution.size(); a++)
            for (size_t t = 0; t < makespan + 1; t++) {
                if (t < solution[a].states.size())
                    paths[t][a] = solution[a].states[t].first.n;
                else
                    paths[t][a] = paths[t - 1][a];
            }
        return paths;
    };

    std::vector<std::vector<Node>> solve(
        const MultiGoalTask& task, const Graph& graph,
        int horizon = std::numeric_limits<int>::max(),
        int execution_horizon = std::numeric_limits<int>::max(),
        double w = 1.5) {
        assert(execution_horizon <= horizon);
        // Initialize data structures
        auto paths = std::vector<std::vector<Node>>(
            {task.starts});  // Tracks robots' current configuration
        auto labels = std::vector<size_t>(
            task.num_robots, 0);  // Tracks robots' job finishing condition
        int max_finished = 0;
        auto target_config = std::vector<Node>();
        for (size_t i = 0; i < task.num_robots; i++) {
            max_finished += task.goals[i].size();
            target_config.push_back(task.goals[i].back());
        }
        int num_finished = 0;
        auto robot_finished = std::vector<bool>(task.num_robots, false);
        // Adaptive horizon
        auto estimated_step_time = std::map<int, double>();
        double local_exploration_rate = exploration_rate;
        std::default_random_engine generator(time(0));
        std::normal_distribution<double> distribution(0, 2);
        // Solve
        while (true) {
            auto start_time = std::chrono::high_resolution_clock::now();
            // Get paths in a window
            auto windowed_paths =
                solve_step(task, graph, paths.back(), labels, horizon, w);
            double time_cost = time_elapsed(start_time) / horizon;
            // Execute paths and count number of finished jobs
            for (size_t t = 1; t < std::min(execution_horizon + 1,
                                            int(windowed_paths.size()));
                 t++) {
                paths.push_back(paths.back());
                for (size_t i = 0; i < task.num_robots; i++) {
                    paths.back()[i] = windowed_paths[t][i];
                    while (paths.back()[i] == task.goals[i][labels[i]] &&
                           labels[i] < task.goals[i].size() - 1) {
                        labels[i]++;
                        num_finished++;
                    }
                }
            }
            // Check if finished
            int real_finished = num_finished;
            if (num_finished >= max_finished - task.num_robots)
                for (size_t i = 0; i < task.num_robots; i++)
                    real_finished += (paths.back()[i] == target_config[i]);
            progress_bar(double(real_finished) / task.target_goal_reaching_num);
            if (real_finished >= task.target_goal_reaching_num ||
                real_finished - num_finished == task.num_robots) {
                progress_bar(1);
                std::cout << std::endl;
                break;
            }
            // Adaptive
            if (adaptive_horizon) {
                // Update data
                for (int i = horizon - 6; i <= horizon + 6; i++) {
                    auto it = estimated_step_time.find(i);
                    if (it == estimated_step_time.end())
                        estimated_step_time.insert(
                            std::make_pair(horizon, time_cost));
                    else
                        it->second =
                            it->second * (1 - learning_rate *
                                                  (1 - abs(i - horizon)) / 6) +
                            time_cost * learning_rate * (1 - abs(i - horizon)) /
                                6;
                }
                // Choose the next horizon
                horizon = std::min_element(estimated_step_time.begin(),
                                           estimated_step_time.end(),
                                           [](const auto& l, const auto& r) {
                                               return l.second < r.second;
                                           })
                              ->first;
                // Explore if needed
                if ((double)rand() / (RAND_MAX) < local_exploration_rate) {
                    horizon += distribution(generator);
                    if (horizon <= 0) horizon = 1;
                    local_exploration_rate =
                        local_exploration_rate * exploration_decay;
                }
                execution_horizon = horizon;
            }
        }
        return paths;
    }

    SUO* suo = nullptr;  // Place for the SUO heuristic
    bool use_suo = false;
    int default_horizon = 5;
    bool adaptive_horizon = false;
    double learning_rate = 0.25;
    double exploration_rate = 0.5;
    double exploration_decay = 0.99;
};