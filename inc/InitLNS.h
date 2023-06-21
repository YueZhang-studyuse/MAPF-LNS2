#pragma once
#include "BasicLNS.h"

enum init_destroy_heuristic { TARGET_BASED, COLLISION_BASED, RANDOM_BASED, INIT_COUNT };

class InitLNS : public BasicLNS
{
public:
    vector<Agent>& agents;
    int num_of_colliding_pairs = 0;
    int num_of_colliding_pairs_windowed = 0;

    int commit_window = -1;

    InitLNS(const Instance& instance, vector<Agent>& agents, double time_limit,
            const string & replan_algo_name, const string & init_destory_name, int neighbor_size, int screen);

    //bool getInitialSolutionBySPC();
    bool run();
    bool runForCommit();
    void writeIterStatsToFile(const string & file_name) const;
    void writeResultToFile(const string & file_name, int sum_of_distances, double preprocessing_time) const;
    string getSolverName() const override { return "InitLNS(" + replan_algo_name + ")"; }

    void printPath() const;
    void printResult();
    void clear(); // delete useless data to save memory

    //for testing purpose
    list<int> conflicts_in_commit_window;
    list<int> conflicts_all;
    bool check_initial = false;
    bool attachInitialSolutionBySPC();

    //add more stats
    map<int,int> num_conflicts_windowed;

    bool time_aware = false;
    bool accept_ontime = false;
    


private:
    string replan_algo_name;
    init_destroy_heuristic init_destroy_strategy = COLLISION_BASED;

    PathTableWC path_table; // 1. stores the paths of all agents in a time-space table;
    // 2. avoid making copies of this variable as much as possible.

    // vector<set<int>> collision_graph; //the old collision graph, witch we do not save conflict time
    
    //collidingSet total_colliding_pairs;

    vector<unordered_map<int,int>> time_collision_graph;
    tuple<int,int,int> earlies_colliding_pairs = make_tuple(INT_MAX,-1,-1); //timestep,a1,a2

    
    vector<int> goal_table;


    // bool runPP();
    // bool runGCBS();
    // bool runPBS();
    bool runTimePP();
    bool updateCollidingPairs(set<pair<int, int>>& colliding_pairs, int agent_id, const Path& path) const;
    //bool updateCollidingPairsInWindow(set<pair<int, int>>& colliding_pairs, set<pair<int, int>>& windowed_colliding_pairs, int agent_id, const Path& path) const;
    bool updateCollidingPairsByTime(unordered_map<pair<int,int>,int> & colliding_pairs, unordered_map<pair<int,int>,int> & windowed_colliding_pairs, int agent_id, const Path& path) const;

    void chooseDestroyHeuristicbyALNS();

    bool generateNeighborByEarlyCollisionGraph();
    bool generateNeighborByTarget();
    bool generateNeighborByEarlyTarget();
    bool generateNeighborRandomly();


    // int findRandomAgent() const;
    int randomWalk(int agent_id);

    void printCollisionGraph() const;

    // static unordered_map<int, set<int>>& findConnectedComponent(const vector<set<int>>& graph, int vertex,
    //         unordered_map<int, set<int>>& sub_graph);
    static unordered_set<int>& findConnectedComponent(const vector<unordered_map<int,int>>& graph, int vertex,
              unordered_set<int>& sub_graph, int max_size);

    bool validatePathTable() const;
};
