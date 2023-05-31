#pragma once
#include "common.h"
#include "SpaceTimeAStar.h"
#include "SIPP.h"


struct Agent
{
    int id;
    SingleAgentSolver* path_planner = nullptr; // start, goal, and heuristics are stored in the path planner
    Path path;

    Agent(const Instance& instance, int id, bool sipp) : id(id)
    {
        if(sipp)
            path_planner = new SIPP(instance, id);
        else
            path_planner = new SpaceTimeAStar(instance, id);
    }
    ~Agent(){ delete path_planner; }

    int getNumOfDelays() const
    {
        return (int) path.size() - 1 - path_planner->my_heuristic[path_planner->start_location];
    }
};

struct CollisionGraphHash
{
    size_t operator()(const tuple<int,int,int>& collding) const
    {
        size_t a1_hash = std::hash<int>()(std::get<0>(collding));
        size_t a2_hash = std::hash<int>()(std::get<1>(collding));
        return a1_hash ^ a2_hash;
    }
};

struct CollisionGraphEqual
{
    bool operator()(const tuple<int,int,int>& collding1, const tuple<int,int,int>& collding2) const
    {
        return std::get<0>(collding1) == std::get<0>(collding2) && std::get<1>(collding1) == std::get<1>(collding2);
    }
};

struct CollisionQueueCmp
{
    size_t operator()(const tuple<int,int,int>& collding1, const tuple<int,int,int>& collding2) const
    {
        if (std::get<2>(collding1) == std::get<2>(collding2))
        {
            return rand() > 0.5; //we give some randanness for conflicts with the same timestep
        }
        return std::get<2>(collding1) > std::get<2>(collding2);
    }
};

typedef unordered_set<tuple<int,int,int>,CollisionGraphHash,CollisionGraphEqual> collidingSet;


struct Neighbor
{
    vector<int> agents;
    int sum_of_costs;
    int old_sum_of_costs;

    //for checking target
    int sum_of_costs_target;
    int old_sum_of_costs_target;

    // set<pair<int, int>> colliding_pairs;  // id1 < id2
    // set<pair<int, int>> old_colliding_pairs;  // id1 < id2
    vector<Path> old_paths;

    // set<pair<int, int>> colliding_pairs_windowed; 
    // set<pair<int, int>> old_colliding_pairs_windowed;

    //maybe it is easier we change it to a map?
    // typedef unordered_set<tuple<int,int,int>,CollisionGraphHash,CollisionGraphEqual> collidingSet;
    collidingSet colliding_pairs;
    collidingSet old_colliding_pairs;
    //we still need windowed because we want to compare 
    collidingSet colliding_pairs_windowed;
    collidingSet old_colliding_pairs_windowed;
};


class BasicLNS
{
public:
    // statistics
    int num_of_failures = 0; // #replanning that fails to find any solutions
    list<IterationStats> iteration_stats; //stats about each iteration
    double runtime = 0;
    double average_group_size = -1;
    int sum_of_costs = 0;

    BasicLNS(const Instance& instance, double time_limit, int neighbor_size, int screen);
    virtual string getSolverName() const = 0;
protected:
    // input params
    const Instance& instance; // avoid making copies of this variable as much as possible
    double time_limit;
    double replan_time_limit; // time limit for replanning
    int neighbor_size;
    int screen;

    // adaptive LNS
    bool ALNS = false;
    double decay_factor = -1;
    double reaction_factor = -1;
    vector<double> destroy_weights;
    int selected_neighbor;

    // helper variables
    high_resolution_clock::time_point start_time;
    Neighbor neighbor;

    void rouletteWheel();
};