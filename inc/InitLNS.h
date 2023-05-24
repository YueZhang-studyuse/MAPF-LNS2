#pragma once
#include "BasicLNS.h"

enum init_destroy_heuristic { TARGET_BASED, COLLISION_BASED, RANDOM_BASED, INIT_COUNT };

// struct CollisionGraphCmpByTime
// {
//     bool operator() (const tuple<int,int,int> &a, const tuple<int,int,int> &b)
//     {
//         // if (a.first == b.first)
//         //     return false;
//         // return a.second > b.second;
//         if (std::get<0>(a) == std::get<0>(b) && std::get<1>(a) == std::get<1>(b))
//         {
//             return false;
//         }
//         return std::get<2>(a) > std::get<2>(b);
//     }
// };

class InitLNS : public BasicLNS
{
public:
    vector<Agent>& agents;
    int num_of_colliding_pairs = 0;
    int num_of_colliding_pairs_windowed = 0;

    int commit_window = -1;

    InitLNS(const Instance& instance, vector<Agent>& agents, double time_limit,
            const string & replan_algo_name, const string & init_destory_name, int neighbor_size, int screen);

    bool getInitialSolutionBySPC();
    bool run();
    void writeIterStatsToFile(const string & file_name) const;
    void writeResultToFile(const string & file_name, int sum_of_distances, double preprocessing_time) const;
    string getSolverName() const override { return "InitLNS(" + replan_algo_name + ")"; }

    void printPath() const;
    void printResult();
    void clear(); // delete useless data to save memory

    //for testing purpose
    list<int> conflicts_in_commit_window;
    list<int> conflicts_all;


private:
    string replan_algo_name;
    init_destroy_heuristic init_destroy_strategy = COLLISION_BASED;

    PathTableWC path_table; // 1. stores the paths of all agents in a time-space table;
    // 2. avoid making copies of this variable as much as possible.

    // vector<set<int>> collision_graph; //the old collision graph, witch we do not save conflict time
    // //maybe we also need to know the conflict time?
    // vector<set<int>> collision_graph_windowed;
    //collision graph with time, <agent1,agent2,time>
    typedef set<tuple<int,int,int>,CollisionGraphCmpByTime> collidingSet;
    collidingSet total_colliding_pairs;
    vector<int> goal_table;


    // bool runPP();
    // bool runGCBS();
    // bool runPBS();
    bool runTimePP();
    bool updateCollidingPairs(set<pair<int, int>>& colliding_pairs, int agent_id, const Path& path) const;
    //bool updateCollidingPairsInWindow(set<pair<int, int>>& colliding_pairs, set<pair<int, int>>& windowed_colliding_pairs, int agent_id, const Path& path) const;
    bool updateCollidingPairsByTime(collidingSet& colliding_pairs, collidingSet& windowed_colliding_pairs, int agent_id, const Path& path) const;

    // void chooseDestroyHeuristicbyALNS();

    // bool generateNeighborByCollisionGraph();
    // bool generateNeighborByTarget();
    // bool generateNeighborRandomly();

    bool generateNeighborByEarlyConflict();

    // int findRandomAgent() const;
    int randomWalk(int agent_id);

    void printCollisionGraph() const;

    static unordered_map<int, set<int>>& findConnectedComponent(const vector<set<int>>& graph, int vertex,
            unordered_map<int, set<int>>& sub_graph);

    bool validatePathTable() const;
};
