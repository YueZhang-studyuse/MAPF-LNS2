#pragma once

#include "common.h"
#include "Instance.h"

class MCP {
public:
    vector<int> to_go;

    MCP(const Instance& instance, int window_size):instance(instance),window_size(window_size){};

    void build(vector<Path*>& paths);
    void clear(void)
    {
        mcp.clear();
        agent_time.clear();
        no_wait_time.clear();
        delay_for.clear();
    };
    void printAll(void);
    void print(int loc);
    void printAgentTime(int num_agents);
    void printAgentNoWaitTime(int num_agents);
    void simulate(vector<Path*>& paths);
private:
    const Instance& instance;

    typedef list<set<int>> Occupy;
    vector<Occupy> mcp;
    vector<int> agent_time;
    vector<vector<int>> no_wait_time;
    vector<Occupy> copy_mcp;
    vector<int> copy_agent_time;
    list<int> unfinished_agents;
    vector<int> delay_for;
    int window_size;

    bool moveAgent(vector<Path>& paths_copy, vector<Path*>& paths, list<int>::iterator& p, int t);
};