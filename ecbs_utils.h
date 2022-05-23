/**
 * @file ecbs_utils.h
 * @author Shuai Han (shuai.han@rutgers.edu)
 * @brief ECBS calling utilities, copied from
 * https://github.com/whoenig/libMultiRobotPlanning/blob/master/example/ecbs.cpp
 * @version 0.1
 * @date 2021-01-12
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <fstream>
#include <iostream>
#include <numeric>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <libMultiRobotPlanning/ecbs.hpp>

#include "task.h"

struct ECBSState {
    ECBSState(int time, int x, int y) : time(time), x(x), y(y) {}
    ECBSState(int time, const Node& n) : time(time), x(n.x), y(n.y) {}

    bool operator==(const ECBSState& s) const {
        return time == s.time && x == s.x && y == s.y;
    }

    bool equalExceptTime(const ECBSState& s) const {
        return x == s.x && y == s.y;
    }

    inline friend std::ostream& operator<<(std::ostream& os,
                                           const ECBSState& s) {
        return os << s.time << ": (" << s.x << "," << s.y << ")";
    }

    int time;
    int x;
    int y;
};

namespace std {
template <>
struct hash<ECBSState> {
    size_t operator()(const ECBSState& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.time);
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        return seed;
    }
};
}  // namespace std

///
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

///

struct ECBSConflict {
    enum Type {
        Vertex,
        Edge,
    };

    int time;
    size_t agent1;
    size_t agent2;
    Type type;

    int x1;
    int y1;
    int x2;
    int y2;

    inline friend std::ostream& operator<<(std::ostream& os,
                                           const ECBSConflict& c) {
        switch (c.type) {
            case Vertex:
                return os << c.time << ": Vertex(" << c.x1 << "," << c.y1
                          << ")";
            case Edge:
                return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << ","
                          << c.x2 << "," << c.y2 << ")";
        }
        return os;
    }
};

struct ECBSVertexConstraint {
    ECBSVertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
    int time;
    int x;
    int y;

    bool operator<(const ECBSVertexConstraint& other) const {
        return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
    }

    bool operator==(const ECBSVertexConstraint& other) const {
        return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
    }

    inline friend std::ostream& operator<<(std::ostream& os,
                                           const ECBSVertexConstraint& c) {
        return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
    }
};

namespace std {
template <>
struct hash<ECBSVertexConstraint> {
    size_t operator()(const ECBSVertexConstraint& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.time);
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        return seed;
    }
};
}  // namespace std

struct ECBSEdgeConstraint {
    ECBSEdgeConstraint(int time, int x1, int y1, int x2, int y2)
        : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
    int time;
    int x1;
    int y1;
    int x2;
    int y2;

    bool operator<(const ECBSEdgeConstraint& other) const {
        return std::tie(time, x1, y1, x2, y2) <
               std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    bool operator==(const ECBSEdgeConstraint& other) const {
        return std::tie(time, x1, y1, x2, y2) ==
               std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    inline friend std::ostream& operator<<(std::ostream& os,
                                           const ECBSEdgeConstraint& c) {
        return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << ","
                  << c.x2 << "," << c.y2 << ")";
    }
};

namespace std {
template <>
struct hash<ECBSEdgeConstraint> {
    size_t operator()(const ECBSEdgeConstraint& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.time);
        boost::hash_combine(seed, s.x1);
        boost::hash_combine(seed, s.y1);
        boost::hash_combine(seed, s.x2);
        boost::hash_combine(seed, s.y2);
        return seed;
    }
};
}  // namespace std

struct ECBSConstraints {
    std::unordered_set<ECBSVertexConstraint> vertexConstraints;
    std::unordered_set<ECBSEdgeConstraint> edgeConstraints;

    void add(const ECBSConstraints& other) {
        vertexConstraints.insert(other.vertexConstraints.begin(),
                                 other.vertexConstraints.end());
        edgeConstraints.insert(other.edgeConstraints.begin(),
                               other.edgeConstraints.end());
    }

    bool overlap(const ECBSConstraints& other) const {
        for (const auto& vc : vertexConstraints) {
            if (other.vertexConstraints.count(vc) > 0) {
                return true;
            }
        }
        for (const auto& ec : edgeConstraints) {
            if (other.edgeConstraints.count(ec) > 0) {
                return true;
            }
        }
        return false;
    }

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

///
class ECBSEnvironment {
   public:
    ECBSEnvironment(size_t dimx, size_t dimy,
                    const std::unordered_set<Node>& obstacles,
                    std::vector<Node> goals)
        : m_dimx(dimx),
          m_dimy(dimy),
          m_obstacles(std::move(obstacles)),
          m_goals(std::move(goals)),
          m_agentIdx(0),
          m_constraints(nullptr),
          m_lastGoalConstraint(-1),
          m_highLevelExpanded(0),
          m_lowLevelExpanded(0) {}

    ECBSEnvironment(const ECBSEnvironment&) = delete;
    ECBSEnvironment& operator=(const ECBSEnvironment&) = delete;

    void setLowLevelContext(size_t agentIdx,
                            const ECBSConstraints* constraints) {
        assert(constraints);
        m_agentIdx = agentIdx;
        m_constraints = constraints;
        m_lastGoalConstraint = -1;
        for (const auto& vc : constraints->vertexConstraints) {
            if (vc.x == m_goals[m_agentIdx].x &&
                vc.y == m_goals[m_agentIdx].y) {
                m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
            }
        }
    }

    int admissibleHeuristic(const ECBSState& s) {
        return std::abs(s.x - m_goals[m_agentIdx].x) +
               std::abs(s.y - m_goals[m_agentIdx].y);
    }

    // low-level
    int focalStateHeuristic(const ECBSState& s, int /*gScore*/,
                            const std::vector<libMultiRobotPlanning::PlanResult<
                                ECBSState, ECBSAction, int>>& solution) {
        int numConflicts = 0;
        for (size_t i = 0; i < solution.size(); ++i) {
            if (i != m_agentIdx && !solution[i].states.empty()) {
                ECBSState state2 = getState(i, solution, s.time);
                if (s.equalExceptTime(state2)) {
                    ++numConflicts;
                }
            }
        }
        return numConflicts;
    }

    // low-level
    int focalTransitionHeuristic(
        const ECBSState& s1a, const ECBSState& s1b, int /*gScoreS1a*/,
        int /*gScoreS1b*/,
        const std::vector<
            libMultiRobotPlanning::PlanResult<ECBSState, ECBSAction, int>>&
            solution) {
        int numConflicts = 0;
        for (size_t i = 0; i < solution.size(); ++i) {
            if (i != m_agentIdx && !solution[i].states.empty()) {
                ECBSState s2a = getState(i, solution, s1a.time);
                ECBSState s2b = getState(i, solution, s1b.time);
                if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) {
                    ++numConflicts;
                }
            }
        }
        return numConflicts;
    }

    // Count all conflicts
    int focalHeuristic(const std::vector<libMultiRobotPlanning::PlanResult<
                           ECBSState, ECBSAction, int>>& solution) {
        int numConflicts = 0;

        int max_t = 0;
        for (const auto& sol : solution) {
            max_t = std::max<int>(max_t, sol.states.size() - 1);
        }

        for (int t = 0; t < max_t; ++t) {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i) {
                ECBSState state1 = getState(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    ECBSState state2 = getState(j, solution, t);
                    if (state1.equalExceptTime(state2)) {
                        ++numConflicts;
                    }
                }
            }
            // drive-drive edge (swap)
            for (size_t i = 0; i < solution.size(); ++i) {
                ECBSState state1a = getState(i, solution, t);
                ECBSState state1b = getState(i, solution, t + 1);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    ECBSState state2a = getState(j, solution, t);
                    ECBSState state2b = getState(j, solution, t + 1);
                    if (state1a.equalExceptTime(state2b) &&
                        state1b.equalExceptTime(state2a)) {
                        ++numConflicts;
                    }
                }
            }
        }
        return numConflicts;
    }

    bool isSolution(const ECBSState& s) {
        return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
               s.time > m_lastGoalConstraint;
    }

    void getNeighbors(const ECBSState& s,
                      std::vector<libMultiRobotPlanning::Neighbor<
                          ECBSState, ECBSAction, int>>& neighbors) {
        // std::cout << "#VC " << constraints.vertexConstraints.size() <<
        // std::endl; for(const auto& vc : constraints.vertexConstraints) {
        //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
        //   std::endl;
        // }
        neighbors.clear();
        {
            ECBSState n(s.time + 1, s.x, s.y);
            if (stateValid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                    libMultiRobotPlanning::Neighbor<ECBSState, ECBSAction, int>(
                        n, ECBSAction::Wait, 1));
            }
        }
        {
            ECBSState n(s.time + 1, s.x - 1, s.y);
            if (stateValid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                    libMultiRobotPlanning::Neighbor<ECBSState, ECBSAction, int>(
                        n, ECBSAction::Left, 1));
            }
        }
        {
            ECBSState n(s.time + 1, s.x + 1, s.y);
            if (stateValid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                    libMultiRobotPlanning::Neighbor<ECBSState, ECBSAction, int>(
                        n, ECBSAction::Right, 1));
            }
        }
        {
            ECBSState n(s.time + 1, s.x, s.y + 1);
            if (stateValid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                    libMultiRobotPlanning::Neighbor<ECBSState, ECBSAction, int>(
                        n, ECBSAction::Up, 1));
            }
        }
        {
            ECBSState n(s.time + 1, s.x, s.y - 1);
            if (stateValid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                    libMultiRobotPlanning::Neighbor<ECBSState, ECBSAction, int>(
                        n, ECBSAction::Down, 1));
            }
        }
    }

    bool getFirstConflict(const std::vector<libMultiRobotPlanning::PlanResult<
                              ECBSState, ECBSAction, int>>& solution,
                          ECBSConflict& result) {
        int max_t = 0;
        for (const auto& sol : solution) {
            max_t = std::max<int>(max_t, sol.states.size() - 1);
        }

        for (int t = 0; t < max_t; ++t) {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i) {
                ECBSState state1 = getState(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    ECBSState state2 = getState(j, solution, t);
                    if (state1.equalExceptTime(state2)) {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = ECBSConflict::Vertex;
                        result.x1 = state1.x;
                        result.y1 = state1.y;
                        // std::cout << "VC " << t << "," << state1.x << "," <<
                        // state1.y << std::endl;
                        return true;
                    }
                }
            }
            // drive-drive edge (swap)
            for (size_t i = 0; i < solution.size(); ++i) {
                ECBSState state1a = getState(i, solution, t);
                ECBSState state1b = getState(i, solution, t + 1);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    ECBSState state2a = getState(j, solution, t);
                    ECBSState state2b = getState(j, solution, t + 1);
                    if (state1a.equalExceptTime(state2b) &&
                        state1b.equalExceptTime(state2a)) {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = ECBSConflict::Edge;
                        result.x1 = state1a.x;
                        result.y1 = state1a.y;
                        result.x2 = state1b.x;
                        result.y2 = state1b.y;
                        return true;
                    }
                }
            }
        }

        return false;
    }

    void createConstraintsFromConflict(
        const ECBSConflict& conflict,
        std::map<size_t, ECBSConstraints>& constraints) {
        if (conflict.type == ECBSConflict::Vertex) {
            ECBSConstraints c1;
            c1.vertexConstraints.emplace(
                ECBSVertexConstraint(conflict.time, conflict.x1, conflict.y1));
            constraints[conflict.agent1] = c1;
            constraints[conflict.agent2] = c1;
        } else if (conflict.type == ECBSConflict::Edge) {
            ECBSConstraints c1;
            c1.edgeConstraints.emplace(
                ECBSEdgeConstraint(conflict.time, conflict.x1, conflict.y1,
                                   conflict.x2, conflict.y2));
            constraints[conflict.agent1] = c1;
            ECBSConstraints c2;
            c2.edgeConstraints.emplace(
                ECBSEdgeConstraint(conflict.time, conflict.x2, conflict.y2,
                                   conflict.x1, conflict.y1));
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
    ECBSState getState(size_t agentIdx,
                       const std::vector<libMultiRobotPlanning::PlanResult<
                           ECBSState, ECBSAction, int>>& solution,
                       size_t t) {
        assert(agentIdx < solution.size());
        if (t < solution[agentIdx].states.size()) {
            return solution[agentIdx].states[t].first;
        }
        assert(!solution[agentIdx].states.empty());
        return solution[agentIdx].states.back().first;
    }

    bool stateValid(const ECBSState& s) {
        assert(m_constraints);
        const auto& con = m_constraints->vertexConstraints;
        return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
               m_obstacles.find(Node(s.x, s.y)) == m_obstacles.end() &&
               con.find(ECBSVertexConstraint(s.time, s.x, s.y)) == con.end();
    }

    bool transitionValid(const ECBSState& s1, const ECBSState& s2) {
        assert(m_constraints);
        const auto& con = m_constraints->edgeConstraints;
        return con.find(ECBSEdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
               con.end();
    }

   private:
    int m_dimx;
    int m_dimy;
    std::unordered_set<Node> m_obstacles;
    std::vector<Node> m_goals;
    size_t m_agentIdx;
    const ECBSConstraints* m_constraints;
    int m_lastGoalConstraint;
    int m_highLevelExpanded;
    int m_lowLevelExpanded;
};

struct ECBSInfo {
    size_t high_level_expanded = 0;
    size_t low_level_expanded = 0;
    size_t cost = 0;
    size_t makespan = 0;
    std::vector<std::vector<size_t>> path;
};

inline ECBSInfo combine_info(std::vector<ECBSInfo>& infos) {
    auto ret_info = ECBSInfo();
    ret_info.path = std::vector<std::vector<size_t>>();
    for (auto& info : infos) {
        ret_info.path.insert(ret_info.path.end(), info.path.begin(),
                             info.path.end());
        ret_info.path.pop_back();
        ret_info.high_level_expanded += info.high_level_expanded;
        ret_info.low_level_expanded += info.low_level_expanded;
    }
    ret_info.path.emplace_back(infos.back().path.back());
    ret_info.makespan = ret_info.path.size() - 1;
    auto robot_cost = std::vector<size_t>(ret_info.path[0].size(), 0);
    for (size_t i = 0; i < ret_info.path.size() - 1; i++)
        for (size_t j = 0; j < ret_info.path[0].size(); j++)
            if (ret_info.path[i][j] != ret_info.path.back()[j])
                robot_cost[j] = i + 1;
    ret_info.cost = std::accumulate(robot_cost.begin(), robot_cost.end(), 0);
    return ret_info;
}
