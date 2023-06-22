#include "MCP.h"


void MCP::simulate(vector<Path*>& paths)
{
    vector<Path> path_copy; 
    path_copy.resize(paths.size());
    copy_agent_time = agent_time;
    copy_mcp = mcp;
    unfinished_agents.clear();
    for (int i = 0; i < paths.size(); i++)
    {
        if (paths[i]->size() >=2)
            unfinished_agents.push_back(i);
        path_copy[i].reserve(paths[i]->size() * 2);
        if (copy_agent_time[i] > 0)
        {
            path_copy[i].push_back(paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]));
        }
    }
    
    for (int t = 0; t <= window_size+1 && !unfinished_agents.empty(); t++) {
        auto old_size = unfinished_agents.size();
        for (auto p = unfinished_agents.begin(); p != unfinished_agents.end();) {

            moveAgent(path_copy, paths, p, t);
        }
        
    }

    for (int i = 0; i < paths.size(); i++)
    {
        if (copy_agent_time[i] != (int) no_wait_time[i].size()){
            path_copy[i].insert(path_copy[i].end()
            , std::next(paths[i]->begin(),no_wait_time[i][copy_agent_time[i]]), paths[i]->end());
        }
        *(paths[i]) = path_copy[i];
    }
}

bool MCP::moveAgent(vector<Path>& paths_copy, vector<Path*>& paths, list<int>::iterator& p, int t)
{
    int i = *p;
    if (paths_copy[i].size() == t + 2)  // we have already made the movement decision for the agent
    {
        ++p;
        return false;
    }
    assert(paths_copy[i].size() == t + 1);
    assert(copy_agent_time[i] <= (int) no_wait_time[i].size());
    if (copy_agent_time[i] == (int) no_wait_time[i].size()) // the agent has reached the last location on its path
    {
        int loc = paths[i]->back().location;
        if (paths_copy[i][t].location == loc)// the agent has reached its goal location
        {
            assert(copy_mcp[loc].front().count(i)>0);
            copy_mcp[loc].front().erase(i);
            if (copy_mcp[loc].front().empty())
                copy_mcp[loc].pop_front();
            p = unfinished_agents.erase(p);
            return true;
        }
        else 
        {
            assert(false);
        }
    }


    // check mcp to determine whether the agent should move or wait
    int loc = paths[i]->at(no_wait_time[i][copy_agent_time[i]]).location;
    assert(!copy_mcp[loc].empty());

    if (t < delay_for[i]){
        ++p;
        return false;
    }

    if (copy_mcp[loc].front().count(i)>0)
    {
        paths_copy[i].push_back(paths[i]->at(no_wait_time[i][copy_agent_time[i]])); // move
        if (copy_agent_time[i] > 0)
        {
            int previous = paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]).location;
            
            if (copy_mcp[previous].front().count(i)==0)
            {
                cout<<"error: previouse location's front has no "<<i<<" :";
                for (auto item : copy_mcp[previous])
                {
                    cout<<"[";
                    for(auto a : item)
                     cout << a << ", ";
                    cout<<"],";
                }
                cout << endl;
            }

            copy_mcp[previous].front().erase(i);
            if (copy_mcp[previous].front().empty())
                copy_mcp[previous].pop_front();
        }
        copy_agent_time[i]++;
        ++p;
        return true;
    }
    assert(copy_mcp[loc].size() > 1);

    if ((*std::next(copy_mcp[loc].begin())).count(i) > 0){// the second agent is i
        // pretend this agent can move: see whether the first agent can move successfully
        paths_copy[i].push_back(paths[i]->at(no_wait_time[i][copy_agent_time[i]])); // move
        bool mcp_pop = false;

        
        int previous = paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]).location;
        assert(copy_mcp[previous].front().count(i) != 0);
        copy_mcp[previous].front().erase(i);
        if (copy_mcp[previous].front().empty()){
            copy_mcp[previous].pop_front();
            mcp_pop = true;
        }
        
        copy_agent_time[i]++;

        for (int first_agent:copy_mcp[loc].front()){
            if ( 
            paths_copy[first_agent].size() == t + 1 &&  // we have not made the movement decision for the first agent
            paths_copy[first_agent][t].location == loc )  // the fist agent is already at loc
            {
                auto p2 = std::find(unfinished_agents.begin(), unfinished_agents.end(), first_agent);
                assert(p2 != unfinished_agents.end());
                auto succ = moveAgent(paths_copy, paths, p2, t);
                
                if (!succ)
                // this agent cannot move
                {
                    paths_copy[i][t + 1] = paths[i]->at(t);
                    copy_agent_time[i]--;
                    int previous = paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]).location;

                    if (mcp_pop)
                    {
                        copy_mcp[previous].push_front(set<int>());

                    }
                    copy_mcp[previous].front().insert(i);
                    ++p;
                    return false;
                }
            }
        }
        ++p;
        return true;

    }
    
    paths_copy[i].push_back(paths_copy[i].back()); // stay still
    ++p; // next agent
    return false;
}


void MCP::build(vector<Path*>& paths)  
{
    //if (options1.debug)
    //    cout << "Start MCP ..." << endl;
    size_t map_size = instance.map_size;
    //if (options1.debug)
    //    cout << "map_size: " << map_size << endl;
    mcp.resize(map_size);
    agent_time.resize(paths.size(), 0);
    to_go.resize(paths.size(), -1);
    size_t max_timestep = 0;

    //if (options1.debug)
    //    cout << "max_timestep = " << max_timestep << endl;

    // Push nodes to MCP
    no_wait_time.resize(paths.size());
    delay_for.resize(paths.size(), 0);
    for (size_t t = 0; t < max_timestep; t++)
    {
        unordered_map<int, set<int>> t_occupy;
        for (int i = 0; i<paths.size();i++)
        {
            if (t < paths[i]->size() &&
                
                (t==0 || paths[i]->at(t).location != paths[i]->at(t-1).location))
            {
                t_occupy[paths[i]->at(t).location].insert(i);
                no_wait_time[i].push_back(t);
            }
        }
        for(auto& o : t_occupy){
            mcp[o.first].push_back(o.second);
            if (o.second.size() > 1 && t <= window_size)
                for (auto a : o.second){
                    if (window_size+1 - t > delay_for[a])
                        delay_for[a] = window_size+1 - t;
                }
        }
    }

    for (int i = 0; i<paths.size();i++)
    {
        assert(!no_wait_time[i].empty());
        if (!no_wait_time[i].empty() && no_wait_time[i][0] == 0)
        {
            agent_time[i] = 1;
            to_go[i] = paths[i]->at(0).location;
        }
    }
}



void MCP::printAll()
{
    cout << "==================== MCP ====================" << endl;
    for (int i = 0; i < mcp.size(); i++)
    {
        if (!mcp[i].empty())
        {
            cout << "[" << i << "]: ";
            auto &last = *(--mcp[i].end());
            for (const auto& p: mcp[i])
            {
                cout<<"[";
                for(auto& o: p){
                    cout<<o<<",";
                }
                cout<<"]";
                if (&p != &last)
                    cout << "->";
                else
                    cout << endl;
            }
        }
    }
    cout << "\n================== MCP END ==================" << endl;
}


void MCP::print(int loc)
{
    cout << "==================== MCP ====================" << endl;
    if (loc < mcp.size() && !mcp[loc].empty())
    {
        cout << "[" << loc << "]: ";
        auto &last = *(--mcp[loc].end());
        for (const auto& p: mcp[loc])
        {
                cout<<"[";
                for(auto& o: p){
                    cout<<o<<",";
                }
                cout<<"]";
                if (&p != &last)
                    cout << "->";
                else
                    cout << endl;
        }
    }
    cout << "\n================== MCP END ==================" << endl;
}


void MCP::printAgentTime(int num_agents)
{
    cout << "==================== Time ====================" << endl;
    for (int i = 0; i < num_agents; i++)
    {
        cout << "Agent " << i << ": " << agent_time[i] << endl;
    }
    cout << "================== End Time ==================" << endl;
}


void MCP::printAgentNoWaitTime(int num_agents)
{
    cout << "==================== Time ====================" << endl;
    for (int i = 0; i < num_agents; i++)
    {
        cout << "Agent " << i << ": ";
        for (int t = 0; t < no_wait_time[i].size(); t++)
            cout << no_wait_time[i][t] << ", ";
        cout << endl;
    }
    cout << "================== End Time ==================" << endl;
}


