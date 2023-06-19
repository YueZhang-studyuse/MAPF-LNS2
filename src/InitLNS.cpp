#include "InitLNS.h"
#include <queue>
#include <algorithm>
#include "GCBS.h"
#include "PBS.h"

InitLNS::InitLNS(const Instance& instance, vector<Agent>& agents, double time_limit,
         const string & replan_algo_name, const string & init_destory_name, int neighbor_size, int screen) :
         BasicLNS(instance, time_limit, neighbor_size, screen), agents(agents), replan_algo_name(replan_algo_name),
         path_table(instance.map_size, agents.size()), time_collision_graph(agents.size()), goal_table(instance.map_size, -1)
 {
     replan_time_limit = time_limit;
     if (init_destory_name == "Adaptive")
     {
         ALNS = true;
         destroy_weights.assign(INIT_COUNT, 1);
         decay_factor = 0.05;
         reaction_factor = 0.05;
     }
     else if (init_destory_name == "Target")
         init_destroy_strategy = TARGET_BASED;
     else if (init_destory_name == "Collision")
         init_destroy_strategy = COLLISION_BASED;
     else if (init_destory_name == "Random")
         init_destroy_strategy = RANDOM_BASED;
     else
     {
         cerr << "Init Destroy heuristic " << init_destory_name << " does not exists. " << endl;
         exit(-1);
     }

     for (auto& i:agents) {
         goal_table[i.path_planner->goal_location] = i.id;
     }

 }

bool InitLNS::run()
{
    start_time = Time::now();
    //bool succ = getInitialSolutionBySPC();
    bool succ = attachInitialSolutionBySPC();
    runtime = ((fsec)(Time::now() - start_time)).count();
    iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, runtime, "PP", 0, num_of_colliding_pairs,num_of_colliding_pairs_windowed);
    if (screen >= 4)
        printPath();
    if (screen >= 1)
    {
        cout << "Iteration " << iteration_stats.size() << ", "
             << "group size = " << neighbor.agents.size() << ", "
             << "colliding pairs = " << num_of_colliding_pairs << ", "
             << "solution cost = " << sum_of_costs << ", "
             << "remaining time = " << time_limit - runtime << endl;
        cout<<"old colliding pairs windowed = "<<num_of_colliding_pairs_windowed<<endl;
    }
    if (runtime >= time_limit and !succ)
    {
        printResult();
        return false;
    }

    vector<Path*> paths(agents.size());
    for (auto i = 0; i < agents.size(); i++)
        paths[i] = &agents[i].path;
    while (runtime < time_limit and num_of_colliding_pairs > 0)
    {
        assert(instance.validateSolution(paths, sum_of_costs, num_of_colliding_pairs));
        // if (ALNS)
        //     chooseDestroyHeuristicbyALNS();

        // switch (init_destroy_strategy)
        // {
        //     case TARGET_BASED:
        //         succ = generateNeighborByTarget();
        //         break;
        //     case COLLISION_BASED:
        //         succ = generateNeighborByCollisionGraph();
        //         break;
        //     case RANDOM_BASED:
        //         succ = generateNeighborRandomly();
        //         break;
        //     default:
        //         cerr << "Wrong neighbor generation strategy" << endl;
        //         exit(-1);
        // }
        //succ = generateNeighborByEarlyConflict(); //my strategy, generate neighbor by earlier conflict first
        succ = generateNeighborByEarlyCollisionGraph();
        
        if(!succ || neighbor.agents.empty())
        {
            runtime = ((fsec)(Time::now() - start_time)).count();
            continue;
        }

        // get colliding pairs
        neighbor.old_colliding_pairs.clear();
        neighbor.old_colliding_pairs_windowed.clear(); //windowed

        for (int a: neighbor.agents)
        {
            for (auto j: time_collision_graph[a]) //since collision graph is already make sure everything is unique (if two pairs seems, then the time is the same)
            {
                //j is key-value pair, while key is the agent, value is the time
                neighbor.old_colliding_pairs[make_pair(min(a, j.first), max(a, j.first))] = j.second;
                if (j.second <= commit_window)
                    neighbor.old_colliding_pairs_windowed[make_pair(min(a, j.first), max(a, j.first))] = j.second;
            }
        }
        if (neighbor.old_colliding_pairs.empty() && neighbor.old_colliding_pairs_windowed.empty()) //no need to replan
        {
            assert(init_destroy_strategy == RANDOM_BASED);
            // if (ALNS) // update destroy heuristics
            // {
            //     destroy_weights[selected_neighbor] = (1 - decay_factor) * destroy_weights[selected_neighbor];
            // }
            runtime = ((fsec)(Time::now() - start_time)).count();
            continue;
        }
        // store the neighbor information
        neighbor.old_paths.resize(neighbor.agents.size());
        neighbor.old_sum_of_costs = 0;

        for (int i = 0; i < (int)neighbor.agents.size(); i++)
        {
            int a = neighbor.agents[i];
            //if (replan_algo_name == "PP" || neighbor.agents.size() == 1)
            neighbor.old_paths[i] = agents[a].path;
            path_table.deletePath(neighbor.agents[i]);
            neighbor.old_sum_of_costs += (int) agents[a].path.size() - 1;
        }
        if (screen >= 2)
        {
            cout << "Neighbors: ";
            for (auto a : neighbor.agents)
                cout << a << ", ";
            cout << endl;
            cout << "Old colliding pairs (" << neighbor.old_colliding_pairs.size() << "): ";
            for (const auto & p : neighbor.old_colliding_pairs)
            {
                cout << "(" << p.first.first << "," << p.first.second << ",:" << p.second << "), ";
            }
            cout << endl;
            cout << "Old colliding pairs winodowed: "<<neighbor.old_colliding_pairs_windowed.size() <<endl;
            for (const auto & p : neighbor.old_colliding_pairs_windowed)
            {
                cout << "(" << p.first.first << "," << p.first.second << ",:" << p.second << "), ";
            }
        }

        // if (replan_algo_name == "PP" || neighbor.agents.size() == 1)
        //     succ = runPP();
        // else if (replan_algo_name == "GCBS")
        //     succ = runGCBS();
        // else if (replan_algo_name == "PBS")
        //     succ = runPBS();
        // else
        // {
        //     cerr << "Wrong replanning strategy" << endl;
        //     exit(-1);
        // }
        succ = runTimePP();

        //     if (ALNS) // update destroy heuristics
        //     {
        //         // if (neighbor.colliding_pairs.size() < neighbor.old_colliding_pairs.size())
        //         //     destroy_weights[selected_neighbor] =
        //         //             reaction_factor * (double)(neighbor.old_colliding_pairs.size() -
        //         //             neighbor.colliding_pairs.size()) // / neighbor.agents.size()
        //         //             + (1 - reaction_factor) * destroy_weights[selected_neighbor];
        //         //my windowed ver
        //         if (neighbor.colliding_pairs_windowed.size() < neighbor.old_colliding_pairs_windowed.size())
        //             //neighbor.colliding_pairs.size() < neighbor.old_colliding_pairs.size())
        //             destroy_weights[selected_neighbor] =
        //                     reaction_factor * (double)(neighbor.old_colliding_pairs_windowed.size() -
        //                     neighbor.colliding_pairs_windowed.size()) // / neighbor.agents.size()
        //                     + (1 - reaction_factor) * destroy_weights[selected_neighbor];
        //         else if (neighbor.colliding_pairs_windowed.size() == neighbor.old_colliding_pairs_windowed.size() &&
        //         neighbor.colliding_pairs.size() < neighbor.old_colliding_pairs.size())
        //             destroy_weights[selected_neighbor] =
        //                     reaction_factor * (double)(neighbor.old_colliding_pairs.size() -
        //                     neighbor.colliding_pairs.size()) // / neighbor.agents.size()
        //                     + (1 - reaction_factor) * destroy_weights[selected_neighbor];
        //         else
        //             destroy_weights[selected_neighbor] =
        //                     (1 - decay_factor) * destroy_weights[selected_neighbor];
        //     }
        if (screen >= 2)
            cout << "New colliding pairs = " << neighbor.colliding_pairs.size() << endl;
        if (screen >= 2)
            cout <<"New windowed colliding pairs = "<< neighbor.colliding_pairs_windowed.size() <<endl;
        if (succ) // update collision graph
        {
            num_of_colliding_pairs += (int)neighbor.colliding_pairs.size() - (int)neighbor.old_colliding_pairs.size();
            num_of_colliding_pairs_windowed  += (int)neighbor.colliding_pairs_windowed.size() - (int)neighbor.old_colliding_pairs_windowed.size();


            for(const auto& collidings : neighbor.old_colliding_pairs)
            {
                // time_collision_graph[collidings.first.first].erase(collidings.first.second);
                // time_collision_graph[collidings.first.second].erase(collidings.first.first);
                int a1 = collidings.first.first;
                int a2 = collidings.first.second;
                time_collision_graph[a1].erase(a2);
                time_collision_graph[a2].erase(a1);
            }
            for(const auto& collidings : neighbor.colliding_pairs)
            {
                // time_collision_graph[collidings.first.first].emplace(collidings.first.second);
                // time_collision_graph[collidings.first.second].emplace(collidings.first.first);
                int a1 = collidings.first.first;
                int a2 = collidings.first.second;
                time_collision_graph[a1][a2]=collidings.second;
                time_collision_graph[a2][a1]=collidings.second;
            }
            //update the earliest
            earlies_colliding_pairs = make_tuple(MAX_TIMESTEP,-1,-1);
            for (int a = 0; a < time_collision_graph.size(); a++)
            {
                for (auto colliding: time_collision_graph[a])
                {
                    if (colliding.second < std::get<0>(earlies_colliding_pairs))
                    {
                        earlies_colliding_pairs = make_tuple(colliding.second,min(a,colliding.first), max(colliding.first,a));
                    }
                }
            }
            if (screen >= 2)
                printCollisionGraph();
        }
        runtime = ((fsec)(Time::now() - start_time)).count();
        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
        if (screen >= 1)
        {
            cout << "Iteration " << iteration_stats.size() << ", "
                    << "group size = " << neighbor.agents.size() << ", "
                    << "colliding pairs = " << num_of_colliding_pairs << ", "
                    << "solution cost = " << sum_of_costs << ", "
                    << "remaining time = " << time_limit - runtime << endl;
            cout<<"old colliding pairs windowed = "<<num_of_colliding_pairs_windowed<<endl;
        }
        iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, runtime, replan_algo_name,
                                    0, num_of_colliding_pairs,num_of_colliding_pairs_windowed);
    }
    conflicts_in_commit_window.push_back(num_of_colliding_pairs_windowed);
    conflicts_all.push_back(num_of_colliding_pairs);

    printResult();
    if (commit_window == -1)
        return (num_of_colliding_pairs == 0);
    return (num_of_colliding_pairs_windowed == 0);
}

bool InitLNS::runForCommit()
{
    start_time = Time::now();
    //bool succ = getInitialSolutionBySPC();
    bool succ = attachInitialSolutionBySPC();
    runtime = ((fsec)(Time::now() - start_time)).count();
    iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, runtime, "PP", 0, num_of_colliding_pairs,num_of_colliding_pairs_windowed);
    if (screen >= 4)
        printPath();
    if (screen >= 1)
    {
        cout << "Iteration " << iteration_stats.size() << ", "
             << "group size = " << neighbor.agents.size() << ", "
             << "colliding pairs = " << num_of_colliding_pairs << ", "
             << "solution cost = " << sum_of_costs << ", "
             << "remaining time = " << time_limit - runtime << endl;
        cout<<"old colliding pairs windowed = "<<num_of_colliding_pairs_windowed<<endl;
    }
    if (runtime >= time_limit and !succ)
    {
        printResult();
        return false;
    }

    vector<Path*> paths(agents.size());
    for (auto i = 0; i < agents.size(); i++)
        paths[i] = &agents[i].path;
    while (runtime < time_limit and num_of_colliding_pairs > 0)
    {
        assert(instance.validateSolution(paths, sum_of_costs, num_of_colliding_pairs));
        // if (ALNS)
        //     chooseDestroyHeuristicbyALNS();

        // switch (init_destroy_strategy)
        // {
        //     case TARGET_BASED:
        //         succ = generateNeighborByTarget();
        //         break;
        //     case COLLISION_BASED:
        //         succ = generateNeighborByCollisionGraph();
        //         break;
        //     case RANDOM_BASED:
        //         succ = generateNeighborRandomly();
        //         break;
        //     default:
        //         cerr << "Wrong neighbor generation strategy" << endl;
        //         exit(-1);
        // }
        //succ = generateNeighborByEarlyConflict(); //my strategy, generate neighbor by earlier conflict first
        succ = generateNeighborByEarlyCollisionGraph();
        
        if(!succ || neighbor.agents.empty())
        {
            runtime = ((fsec)(Time::now() - start_time)).count();
            continue;
        }

        // get colliding pairs
        neighbor.old_colliding_pairs.clear();
        neighbor.old_colliding_pairs_windowed.clear(); //windowed

        for (int a: neighbor.agents)
        {
            for (auto j: time_collision_graph[a]) //since collision graph is already make sure everything is unique (if two pairs seems, then the time is the same)
            {
                //j is key-value pair, while key is the agent, value is the time
                neighbor.old_colliding_pairs[make_pair(min(a, j.first), max(a, j.first))] = j.second;
                if (j.second <= commit_window)
                    neighbor.old_colliding_pairs_windowed[make_pair(min(a, j.first), max(a, j.first))] = j.second;
            }
        }
        if (neighbor.old_colliding_pairs.empty() && neighbor.old_colliding_pairs_windowed.empty()) //no need to replan
        {
            assert(init_destroy_strategy == RANDOM_BASED);
            // if (ALNS) // update destroy heuristics
            // {
            //     destroy_weights[selected_neighbor] = (1 - decay_factor) * destroy_weights[selected_neighbor];
            // }
            runtime = ((fsec)(Time::now() - start_time)).count();
            continue;
        }
        // store the neighbor information
        neighbor.old_paths.resize(neighbor.agents.size());
        neighbor.old_sum_of_costs = 0;

        for (int i = 0; i < (int)neighbor.agents.size(); i++)
        {
            int a = neighbor.agents[i];
            //if (replan_algo_name == "PP" || neighbor.agents.size() == 1)
            neighbor.old_paths[i] = agents[a].path;
            path_table.deletePath(neighbor.agents[i]);
            neighbor.old_sum_of_costs += (int) agents[a].path.size() - 1;
        }
        if (screen >= 2)
        {
            cout << "Neighbors: ";
            for (auto a : neighbor.agents)
                cout << a << ", ";
            cout << endl;
            cout << "Old colliding pairs (" << neighbor.old_colliding_pairs.size() << "): ";
            for (const auto & p : neighbor.old_colliding_pairs)
            {
                cout << "(" << p.first.first << "," << p.first.second << ",:" << p.second << "), ";
            }
            cout << endl;
            cout << "Old colliding pairs winodowed: "<<neighbor.old_colliding_pairs_windowed.size() <<endl;
            for (const auto & p : neighbor.old_colliding_pairs_windowed)
            {
                cout << "(" << p.first.first << "," << p.first.second << ",:" << p.second << "), ";
            }
        }

        // if (replan_algo_name == "PP" || neighbor.agents.size() == 1)
        //     succ = runPP();
        // else if (replan_algo_name == "GCBS")
        //     succ = runGCBS();
        // else if (replan_algo_name == "PBS")
        //     succ = runPBS();
        // else
        // {
        //     cerr << "Wrong replanning strategy" << endl;
        //     exit(-1);
        // }
        succ = runTimePP();

        //     if (ALNS) // update destroy heuristics
        //     {
        //         // if (neighbor.colliding_pairs.size() < neighbor.old_colliding_pairs.size())
        //         //     destroy_weights[selected_neighbor] =
        //         //             reaction_factor * (double)(neighbor.old_colliding_pairs.size() -
        //         //             neighbor.colliding_pairs.size()) // / neighbor.agents.size()
        //         //             + (1 - reaction_factor) * destroy_weights[selected_neighbor];
        //         //my windowed ver
        //         if (neighbor.colliding_pairs_windowed.size() < neighbor.old_colliding_pairs_windowed.size())
        //             //neighbor.colliding_pairs.size() < neighbor.old_colliding_pairs.size())
        //             destroy_weights[selected_neighbor] =
        //                     reaction_factor * (double)(neighbor.old_colliding_pairs_windowed.size() -
        //                     neighbor.colliding_pairs_windowed.size()) // / neighbor.agents.size()
        //                     + (1 - reaction_factor) * destroy_weights[selected_neighbor];
        //         else if (neighbor.colliding_pairs_windowed.size() == neighbor.old_colliding_pairs_windowed.size() &&
        //         neighbor.colliding_pairs.size() < neighbor.old_colliding_pairs.size())
        //             destroy_weights[selected_neighbor] =
        //                     reaction_factor * (double)(neighbor.old_colliding_pairs.size() -
        //                     neighbor.colliding_pairs.size()) // / neighbor.agents.size()
        //                     + (1 - reaction_factor) * destroy_weights[selected_neighbor];
        //         else
        //             destroy_weights[selected_neighbor] =
        //                     (1 - decay_factor) * destroy_weights[selected_neighbor];
        //     }
        if (screen >= 2)
            cout << "New colliding pairs = " << neighbor.colliding_pairs.size() << endl;
        if (screen >= 2)
            cout <<"New windowed colliding pairs = "<< neighbor.colliding_pairs_windowed.size() <<endl;
        if (succ) // update collision graph
        {
            num_of_colliding_pairs += (int)neighbor.colliding_pairs.size() - (int)neighbor.old_colliding_pairs.size();
            num_of_colliding_pairs_windowed  += (int)neighbor.colliding_pairs_windowed.size() - (int)neighbor.old_colliding_pairs_windowed.size();


            for(const auto& collidings : neighbor.old_colliding_pairs)
            {
                // time_collision_graph[collidings.first.first].erase(collidings.first.second);
                // time_collision_graph[collidings.first.second].erase(collidings.first.first);
                int a1 = collidings.first.first;
                int a2 = collidings.first.second;
                time_collision_graph[a1].erase(a2);
                time_collision_graph[a2].erase(a1);
            }
            for(const auto& collidings : neighbor.colliding_pairs)
            {
                // time_collision_graph[collidings.first.first].emplace(collidings.first.second);
                // time_collision_graph[collidings.first.second].emplace(collidings.first.first);
                int a1 = collidings.first.first;
                int a2 = collidings.first.second;
                time_collision_graph[a1][a2]=collidings.second;
                time_collision_graph[a2][a1]=collidings.second;
            }
            //update the earliest
            earlies_colliding_pairs = make_tuple(MAX_TIMESTEP,-1,-1);
            for (int a = 0; a < time_collision_graph.size(); a++)
            {
                for (auto colliding: time_collision_graph[a])
                {
                    if (colliding.second < std::get<0>(earlies_colliding_pairs))
                    {
                        earlies_colliding_pairs = make_tuple(colliding.second,min(a,colliding.first), max(colliding.first,a));
                    }
                }
            }
            if (screen >= 2)
                printCollisionGraph();
        }
        runtime = ((fsec)(Time::now() - start_time)).count();
        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
        if (screen >= 1)
        {
            cout << "Iteration " << iteration_stats.size() << ", "
                    << "group size = " << neighbor.agents.size() << ", "
                    << "colliding pairs = " << num_of_colliding_pairs << ", "
                    << "solution cost = " << sum_of_costs << ", "
                    << "remaining time = " << time_limit - runtime << endl;
            cout<<"old colliding pairs windowed = "<<num_of_colliding_pairs_windowed<<endl;
        }
        iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, runtime, replan_algo_name,
                                    0, num_of_colliding_pairs,num_of_colliding_pairs_windowed);
    }
    conflicts_in_commit_window.push_back(num_of_colliding_pairs_windowed);
    conflicts_all.push_back(num_of_colliding_pairs);

    printResult();
    if (commit_window == -1)
        return (num_of_colliding_pairs == 0);
    return (num_of_colliding_pairs_windowed == 0);
}

// bool InitLNS::runGCBS()
// {
//     vector<SingleAgentSolver*> search_engines;
//     search_engines.reserve(neighbor.agents.size());
//     for (int i : neighbor.agents)
//     {
//         search_engines.push_back(agents[i].path_planner);
//     }

//     // build path tables
//     vector<PathTable> path_tables(neighbor.agents.size(), PathTable(instance.map_size));
//     for (int i = 0; i < (int)neighbor.agents.size(); i++)
//     {
//         int agent_id = neighbor.agents[i];
//         for (int j = 0; j < instance.getDefaultNumberOfAgents(); j++)
//         {
//             if (j != agent_id and collision_graph[agent_id].count(j) == 0)
//                 path_tables[i].insertPath(j, agents[j].path);
//         }
//     }

//     GCBS gcbs(search_engines, screen - 1, &path_tables);
//     gcbs.setDisjointSplitting(false);
//     gcbs.setBypass(true);
//     gcbs.setTargetReasoning(true);

//     runtime = ((fsec)(Time::now() - start_time)).count();
//     double T = time_limit - runtime;
//     if (!iteration_stats.empty()) // replan
//         T = min(T, replan_time_limit);
//     gcbs.solve(T);
//     if (gcbs.best_node->colliding_pairs < (int) neighbor.old_colliding_pairs.size()) // accept new paths
//     {
//         auto id = neighbor.agents.begin();
//         neighbor.colliding_pairs.clear();
//         for (size_t i = 0; i < neighbor.agents.size(); i++)
//         {
//             agents[*id].path = *gcbs.paths[i];
//             updateCollidingPairs(neighbor.colliding_pairs, agents[*id].id, agents[*id].path);
//             path_table.insertPath(agents[*id].id, agents[*id].path);
//             ++id;
//         }
//         neighbor.sum_of_costs = gcbs.best_node->sum_of_costs;
//         return true;
//     }
//     else // stick to old paths
//     {
//         if (!neighbor.old_paths.empty())
//         {
//             for (int id : neighbor.agents)
//             {
//                 path_table.insertPath(agents[id].id, agents[id].path);
//             }
//             neighbor.sum_of_costs = neighbor.old_sum_of_costs;
//         }
//         num_of_failures++;
//         return false;
//     }
// }
// bool InitLNS::runPBS()
// {
//     vector<SingleAgentSolver*> search_engines;
//     search_engines.reserve(neighbor.agents.size());
//     vector<const Path*> initial_paths;
//     initial_paths.reserve(neighbor.agents.size());
//     for (int i : neighbor.agents)
//     {
//         search_engines.push_back(agents[i].path_planner);
//         initial_paths.push_back(&agents[i].path);
//     }

//     PBS pbs(search_engines, path_table, screen - 1);
//     // pbs.setInitialPath(initial_paths);
//     runtime = ((fsec)(Time::now() - start_time)).count();
//     double T = time_limit - runtime;
//     if (!iteration_stats.empty()) // replan
//         T = min(T, replan_time_limit);
//     bool succ = pbs.solve(T, (int)neighbor.agents.size(), neighbor.old_colliding_pairs.size());
//     if (succ and pbs.best_node->getCollidingPairs() < (int) neighbor.old_colliding_pairs.size()) // accept new paths
//     {
//         auto id = neighbor.agents.begin();
//         neighbor.colliding_pairs.clear();
//         for (size_t i = 0; i < neighbor.agents.size(); i++)
//         {
//             agents[*id].path = *pbs.paths[i];
//             updateCollidingPairs(neighbor.colliding_pairs, agents[*id].id, agents[*id].path);
//             path_table.insertPath(agents[*id].id);
//             ++id;
//         }
//         assert(neighbor.colliding_pairs.size() == pbs.best_node->getCollidingPairs());
//         neighbor.sum_of_costs = pbs.best_node->sum_of_costs;
//         return true;
//     }
//     else // stick to old paths
//     {
//         if (!neighbor.old_paths.empty())
//         {
//             for (int id : neighbor.agents)
//             {
//                 path_table.insertPath(agents[id].id);
//             }
//             neighbor.sum_of_costs = neighbor.old_sum_of_costs;
//         }
//         num_of_failures++;
//         return false;
//     }
// }
// bool InitLNS::runPP()
// {
//     auto shuffled_agents = neighbor.agents;
//     std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
//     if (screen >= 2) {
//         cout<<"Neighbors_set: ";
//         for (auto id : shuffled_agents)
//             cout << id << ", ";
//         cout << endl;
//     }
//     int remaining_agents = (int)shuffled_agents.size();
//     auto p = shuffled_agents.begin();
//     neighbor.sum_of_costs = 0;
//     neighbor.colliding_pairs.clear();
//     runtime = ((fsec)(Time::now() - start_time)).count();
//     double T = min(time_limit - runtime, replan_time_limit);
//     auto time = Time::now();
//     ConstraintTable constraint_table(instance.num_of_cols, instance.map_size, nullptr, &path_table);
//     while (p != shuffled_agents.end() && ((fsec)(Time::now() - time)).count() < T)
//     {
//         int id = *p;
//         agents[id].path = agents[id].path_planner->findPath(constraint_table);
//         assert(!agents[id].path.empty() && agents[id].path.back().location == agents[id].path_planner->goal_location);
//         if (agents[id].path_planner->num_collisions > 0)
//             updateCollidingPairs(neighbor.colliding_pairs, agents[id].id, agents[id].path);
//         assert(agents[id].path_planner->num_collisions > 0 or
//             !updateCollidingPairs(neighbor.colliding_pairs, agents[id].id, agents[id].path));
//         neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
//         remaining_agents--;
//         if (screen >= 3)
//         {
//             runtime = ((fsec)(Time::now() - start_time)).count();
//             cout << "After agent " << id << ": Remaining agents = " << remaining_agents <<
//                  ", colliding pairs = " << neighbor.colliding_pairs.size() <<
//                  ", LL nodes = " << agents[id].path_planner->getNumExpanded() <<
//                  ", remaining time = " << time_limit - runtime << " seconds. " << endl;
//         }
//         if (neighbor.colliding_pairs.size() >= neighbor.old_colliding_pairs.size())
//             break;
//         path_table.insertPath(agents[id].id, agents[id].path);
//         ++p;
//     }
//     if (p == shuffled_agents.end() && neighbor.colliding_pairs.size() <= neighbor.old_colliding_pairs.size()) // accept new paths
//     {
//         return true;
//     }
//     else // stick to old paths
//     {
//         if (p != shuffled_agents.end())
//             num_of_failures++;
//         auto p2 = shuffled_agents.begin();
//         while (p2 != p)
//         {
//             int a = *p2;
//             path_table.deletePath(agents[a].id);
//             ++p2;
//         }
//         if (!neighbor.old_paths.empty())
//         {
//             p2 = neighbor.agents.begin();
//             for (int i = 0; i < (int)neighbor.agents.size(); i++)
//             {
//                 int a = *p2;
//                 agents[a].path = neighbor.old_paths[i];
//                 path_table.insertPath(agents[a].id);
//                 ++p2;
//             }
//             neighbor.sum_of_costs = neighbor.old_sum_of_costs;
//         }
//         return false;
//     }
// }

bool InitLNS::runTimePP()
{
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
    if (screen >= 2) {
        cout<<"Neighbors_set: ";
        for (auto id : shuffled_agents)
            cout << id << ", ";
        cout << endl;
    }
    int remaining_agents = (int)shuffled_agents.size();
    auto p = shuffled_agents.begin();
    neighbor.sum_of_costs = 0;
    neighbor.colliding_pairs.clear();

    runtime = ((fsec)(Time::now() - start_time)).count();
    double T = min(time_limit - runtime, replan_time_limit);
    auto time = Time::now();

    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size, nullptr, &path_table);

    while (p != shuffled_agents.end() && ((fsec)(Time::now() - time)).count() < T)
    {
        int id = *p;
        agents[id].path_planner->commit_window = commit_window;
        agents[id].path = agents[id].path_planner->findPath(constraint_table);
        if (agents[id].path.empty() || agents[id].path.back().location != agents[id].path_planner->goal_location)
            break;
        assert(!agents[id].path.empty() && agents[id].path.back().location == agents[id].path_planner->goal_location);
        if (agents[id].path_planner->num_collisions > 0)
            updateCollidingPairsByTime(neighbor.colliding_pairs,neighbor.colliding_pairs_windowed, agents[id].id, agents[id].path);
        assert(agents[id].path_planner->num_collisions > 0 or
            !updateCollidingPairsByTime(neighbor.colliding_pairs,neighbor.colliding_pairs_windowed, agents[id].id, agents[id].path));
        neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
        remaining_agents--;
        if (screen >= 3)
        {
            runtime = ((fsec)(Time::now() - start_time)).count();
            cout << "After agent " << id << ": Remaining agents = " << remaining_agents <<
                 ", colliding pairs = " << neighbor.colliding_pairs.size() + neighbor.colliding_pairs_windowed.size() <<
                 ", windowed colliding pairs = " << neighbor.colliding_pairs_windowed.size() <<
                 ", LL nodes = " << agents[id].path_planner->getNumExpanded() <<
                 ", remaining time = " << time_limit - runtime << " seconds. " << endl;
        }
        // for (auto c: neighbor.colliding_pairs)
        // {
        //     int a1,a2,t;
        //     tie(a1,a2,t) = c;
        //     if (a1 == 189 || a1 == 218 || a2 == 189 || a2 == 218)
        //         cout<<"test for collidings "<<a1<<" "<<a2<<" "<<t<<" , ";
        //     cout<<endl;
        // }
        if (neighbor.colliding_pairs_windowed.size() > neighbor.old_colliding_pairs_windowed.size())
            break;
        if (neighbor.colliding_pairs_windowed.size() == neighbor.old_colliding_pairs_windowed.size() && neighbor.colliding_pairs.size() >= neighbor.old_colliding_pairs.size())
            break;
        path_table.insertPath(agents[id].id, agents[id].path);
        ++p;
    }
    if (p == shuffled_agents.end() && neighbor.colliding_pairs_windowed.size() < neighbor.old_colliding_pairs_windowed.size()) // accept new paths
    {
        return true;
    }
    if (p == shuffled_agents.end() && neighbor.colliding_pairs_windowed.size() == neighbor.old_colliding_pairs_windowed.size() && neighbor.colliding_pairs.size() <= neighbor.old_colliding_pairs.size()) // accept new paths
    {
        return true;
    }
    else // stick to old paths
    {
        if (p != shuffled_agents.end())
            num_of_failures++;
        auto p2 = shuffled_agents.begin();
        while (p2 != p)
        {
            int a = *p2;
            path_table.deletePath(agents[a].id);
            ++p2;
        }
        if (!neighbor.old_paths.empty())
        {
            p2 = neighbor.agents.begin();
            for (int i = 0; i < (int)neighbor.agents.size(); i++)
            {
                int a = *p2;
                agents[a].path = neighbor.old_paths[i];
                path_table.insertPath(agents[a].id);
                ++p2;
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        return false;
    }
}

bool InitLNS::attachInitialSolutionBySPC()
{
    unordered_map<pair<int,int>,int> colliding_pairs;
    unordered_map<pair<int,int>,int> colliding_pairs_windowed;
    neighbor.agents.clear();
    neighbor.agents.reserve(agents.size());
    sum_of_costs = 0;
    auto goals = instance.getGoals();
    for (int i = 0; i < (int)agents.size(); i++)
    {
        if (agents[i].path.empty())
            neighbor.agents.push_back(i);
        else //single agent shortest path
        {
            if (agents[i].path.back().location != goals[i])
            {
                //attach single agent shorest path
                list<int> locations;
                for (auto l: agents[i].path)
                {
                    locations.push_back(l.location);
                }
                int curr = locations.back();
                while (curr != goals[i])
                {
                    auto neighbors_location = instance.getNeighbors(curr);
                    int next_loc = curr;
                    for (auto n: neighbors_location)
                    {
                        if (agents[i].path_planner->my_heuristic[n] < agents[i].path_planner->my_heuristic[curr])
                            next_loc = n; //we simply do this because our heuriscit is computed by dijkstra
                    }
                    locations.push_back(next_loc);
                    curr = next_loc;
                }
                //push locations and path together
                agents[i].path.resize(locations.size());
                int index = 0;
                for (auto l: locations)
                {
                    agents[i].path[index] = PathEntry(l);
                    index++;
                }

            }
            updateCollidingPairsByTime(colliding_pairs,colliding_pairs_windowed, agents[i].id, agents[i].path);
            sum_of_costs += (int)agents[i].path.size() - 1;
            path_table.insertPath(agents[i].id, agents[i].path);
        }
    }
    int remaining_agents = (int)neighbor.agents.size();
    //start to plan for agents that have empty path
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size, nullptr, &path_table);
    for (auto id : neighbor.agents)
    {
        agents[id].path = agents[id].path_planner->findPath(constraint_table);
        assert(!agents[id].path.empty() && agents[id].path.back().location == agents[id].path_planner->goal_location);
        if (agents[id].path_planner->num_collisions > 0)
            updateCollidingPairsByTime(colliding_pairs,colliding_pairs_windowed, agents[id].id, agents[id].path); //windowed

        sum_of_costs += (int)agents[id].path.size() - 1;
        remaining_agents--;
        path_table.insertPath(agents[id].id, agents[id].path);
        runtime = ((fsec)(Time::now() - start_time)).count();
        if (screen >= 3)
        {
            cout << "After agent " << id << ": Remaining agents = " << remaining_agents <<
                 ", colliding pairs = " << colliding_pairs.size() <<
                 ", LL nodes = " << agents[id].path_planner->getNumExpanded() <<
                 ", remaining time = " << time_limit - runtime << " seconds. " << endl;
            cout << "windowed colliding pairs "<<colliding_pairs_windowed.size()<<endl;
        }
        if (runtime > time_limit)
            break;
    }

    num_of_colliding_pairs_windowed = colliding_pairs_windowed.size();
    num_of_colliding_pairs = colliding_pairs.size();
    for(const auto& collidings : colliding_pairs) //update what in remained into total collision graph
    {
        //we need to update by time
        // auto iter = total_colliding_pairs.find(collidings);
        // if (iter == total_colliding_pairs.end())
        // {
        //     total_colliding_pairs.emplace(collidings);
        // }
        // else
        // {
        //     if (std::get<2>(collidings) < std::get<2>(*iter))
        //     {
        //         total_colliding_pairs.erase(iter);
        //         total_colliding_pairs.emplace(collidings);
        //     }
        // }
        auto agent_pair = collidings.first;
        int time = collidings.second;
        if (time_collision_graph[agent_pair.first].find(agent_pair.second) != time_collision_graph[agent_pair.first].end())
        {
            int current_time = time_collision_graph[agent_pair.first][agent_pair.second];
            if (time < current_time)
            {
                time_collision_graph[agent_pair.first][agent_pair.second] = time;
            }
        }
        else
        {
            time_collision_graph[agent_pair.first][agent_pair.second] = time;
        }

        if (time_collision_graph[agent_pair.second].find(agent_pair.first) != time_collision_graph[agent_pair.second].end())
        {
            int current_time = time_collision_graph[agent_pair.second][agent_pair.first];
            if (time < current_time)
            {
                time_collision_graph[agent_pair.second][agent_pair.first] = time;
            }
        }
        else
        {
            time_collision_graph[agent_pair.second][agent_pair.first] = time;
        }

        if (time < std::get<0>(earlies_colliding_pairs))//update the earlies record
        {
            earlies_colliding_pairs = make_tuple(time,agent_pair.first,agent_pair.second);
        }
    }

    if (screen >= 2)
        printCollisionGraph();
    
    return remaining_agents == 0;
}

// bool InitLNS::getInitialSolutionBySPC() //solely by individual shortest path
// {
//     neighbor.agents.clear();
//     neighbor.agents.reserve(agents.size());
//     sum_of_costs = 0;
//     for (int i = 0; i < (int)agents.size(); i++)
//     {
//         if (agents[i].path.empty())
//             neighbor.agents.push_back(i);
//         else
//         {
//             sum_of_costs += (int)agents[i].path.size() - 1;
//             path_table.insertPath(agents[i].id, agents[i].path);
//         }
//     }
//     int remaining_agents = (int)neighbor.agents.size();
//     ConstraintTable constraint_table(instance.num_of_cols, instance.map_size, nullptr); //not use any path now, because we only want single agent path
//     // set<pair<int, int>> colliding_pairs;
//     // set<pair<int, int>> colliding_pairs_windowed;
//     unordered_map<pair<int,int>,int> colliding_pairs;
//     unordered_map<pair<int,int>,int> colliding_pairs_windowed;

//     for (auto id : neighbor.agents)
//     {
//         agents[id].path_planner->commit_window = commit_window;
//         agents[id].path = agents[id].path_planner->findPath(constraint_table);
//         assert(!agents[id].path.empty() && agents[id].path.back().location == agents[id].path_planner->goal_location);
//         //check all because we are using single agent shorest path
//         updateCollidingPairsByTime(colliding_pairs,colliding_pairs_windowed, agents[id].id, agents[id].path); //windowed

//         sum_of_costs += (int)agents[id].path.size() - 1;
//         remaining_agents--;
//         path_table.insertPath(agents[id].id, agents[id].path);
//         runtime = ((fsec)(Time::now() - start_time)).count();
//         if (screen >= 3)
//         {
//             cout << "After agent " << id << ": Remaining agents = " << remaining_agents <<
//                  ", colliding pairs = " << colliding_pairs.size() + colliding_pairs_windowed.size() <<
//                  ", LL nodes = " << agents[id].path_planner->getNumExpanded() <<
//                  ", remaining time = " << time_limit - runtime << " seconds. " << endl;
//             cout << "windowed colliding pairs "<<colliding_pairs_windowed.size()<<endl;
//         }
//         if (runtime > time_limit)
//             break;
//     }

//     num_of_colliding_pairs_windowed = colliding_pairs_windowed.size();
//     num_of_colliding_pairs = colliding_pairs.size() + num_of_colliding_pairs_windowed;
//     for(const auto& collidings : colliding_pairs) //update what in remained into total colliding pairs
//     {
//         //we need to update by time
//         auto iter = total_colliding_pairs.find(collidings);
//         if (iter == total_colliding_pairs.end())
//         {
//             total_colliding_pairs.emplace(collidings);
//         }
//         else
//         {
//             if (std::get<2>(collidings) < std::get<2>(*iter))
//             {
//                 total_colliding_pairs.erase(iter);
//                 total_colliding_pairs.emplace(collidings);
//             }
//         }
//         //also insert to pq, no need to check for update
//         //colliding_pq.push(collidings);
//     }
//     for(const auto& collidings : colliding_pairs_windowed) //update what in windowed into total colliding pairs
//     {
//         //we need to update by time
//         auto iter = total_colliding_pairs.find(collidings);
//         if (iter == total_colliding_pairs.end())
//         {
//             total_colliding_pairs.emplace(collidings);
//         }
//         else
//         {
//             if (std::get<2>(collidings) < std::get<2>(*iter))
//             {
//                 total_colliding_pairs.erase(iter);
//                 total_colliding_pairs.emplace(collidings);
//             }
//         }
//         //colliding_pq.push(collidings);
//     }

//     if (screen >= 2)
//         printCollisionGraph();
//     return remaining_agents == 0;
// }
//to keep track of conflicts and solutions in each window, we need to keep track of the time and collision pairs
// bool InitLNS::getInitialSolution()
// {
//     neighbor.agents.clear();
//     neighbor.agents.reserve(agents.size());
//     sum_of_costs = 0;
//     for (int i = 0; i < (int)agents.size(); i++)
//     {
//         if (agents[i].path.empty())
//             neighbor.agents.push_back(i);
//         else
//         {
//             sum_of_costs += (int)agents[i].path.size() - 1;
//             path_table.insertPath(agents[i].id, agents[i].path);
//         }
//     }
//     int remaining_agents = (int)neighbor.agents.size();
//     std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
//     ConstraintTable constraint_table(instance.num_of_cols, instance.map_size, nullptr, &path_table);
//     set<pair<int, int>> colliding_pairs;
//     set<pair<int, int>> colliding_pairs_windowed;
//     for (auto id : neighbor.agents)
//     {
//         agents[id].path = agents[id].path_planner->findPath(constraint_table);
//         assert(!agents[id].path.empty() && agents[id].path.back().location == agents[id].path_planner->goal_location);
//         if (agents[id].path_planner->num_collisions > 0)
//             //updateCollidingPairs(colliding_pairs, agents[id].id, agents[id].path); 
//             updateCollidingPairsInWindow(colliding_pairs,colliding_pairs_windowed, agents[id].id, agents[id].path); //windowed
//         sum_of_costs += (int)agents[id].path.size() - 1;
//         remaining_agents--;
//         path_table.insertPath(agents[id].id, agents[id].path);
//         runtime = ((fsec)(Time::now() - start_time)).count();
//         if (screen >= 3)
//         {
//             cout << "After agent " << id << ": Remaining agents = " << remaining_agents <<
//                  ", colliding pairs = " << colliding_pairs.size() <<
//                  ", LL nodes = " << agents[id].path_planner->getNumExpanded() <<
//                  ", remaining time = " << time_limit - runtime << " seconds. " << endl;
//             cout << "windowed colliding pairs "<<colliding_pairs_windowed.size()<<endl;
//         }
//         if (runtime > time_limit)
//             break;
//     }

//     num_of_colliding_pairs = colliding_pairs.size();
//     num_of_colliding_pairs_windowed = colliding_pairs_windowed.size();
//     for(const auto& agent_pair : colliding_pairs)
//     {
//         collision_graph[agent_pair.first].emplace(agent_pair.second);
//         collision_graph[agent_pair.second].emplace(agent_pair.first);
//         if (colliding_pairs_windowed.find(agent_pair) != colliding_pairs_windowed.end())
//         {
//             collision_graph_windowed[agent_pair.first].emplace(agent_pair.second);
//             collision_graph_windowed[agent_pair.second].emplace(agent_pair.first);
//         }
//     }
//     if (screen >= 2)
//         printCollisionGraph();
//     return remaining_agents == 0;
// }

// return true if the new p[ath has collisions;
bool InitLNS::updateCollidingPairs(set<pair<int, int>>& colliding_pairs, int agent_id, const Path& path) const
{
    bool succ = false;
    if (path.size() < 2)
        return succ;
    for (int t = 1; t < (int)path.size(); t++)
    {
        int from = path[t - 1].location;
        int to = path[t].location;
        if ((int)path_table.table[to].size() > t) // vertex conflicts
        {
            for (auto id : path_table.table[to][t])
            {
                succ = true;
                colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
            }
        }
        if (from != to && path_table.table[to].size() >= t && path_table.table[from].size() > t) // edge conflicts
        {
            for (auto a1 : path_table.table[to][t - 1])
            {
                for (auto a2: path_table.table[from][t])
                {
                    if (a1 == a2)
                    {
                        succ = true;
                        colliding_pairs.emplace(min(agent_id, a1), max(agent_id, a1));
                        break;
                    }
                }
            }
        }
        //auto id = getAgentWithTarget(to, t);
        //if (id >= 0) // this agent traverses the target of another agent
        //    colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
        if (!path_table.goals.empty() && path_table.goals[to] < t) // target conflicts
        { // this agent traverses the target of another agent
            for (auto id : path_table.table[to][path_table.goals[to]]) // look at all agents at the goal time
            {
                if (agents[id].path.back().location == to) // if agent id's goal is to, then this is the agent we want
                {
                    succ = true;
                    colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
                    break;
                }
            }
        }
    }
    int goal = path.back().location; // target conflicts - some other agent traverses the target of this agent
    for (int t = (int)path.size(); t < path_table.table[goal].size(); t++)
    {
        for (auto id : path_table.table[goal][t])
        {
            succ = true;
            colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
        }
    }
    return succ;
}

// return true if the new p[ath has collisions;
bool InitLNS::updateCollidingPairsByTime(unordered_map<pair<int,int>,int> & colliding_pairs, unordered_map<pair<int,int>,int> & windowed_colliding_pairs, int agent_id, const Path& path) const
{
    bool succ = false;
    if (path.size() < 2)
        return succ;
    for (int t = 1; t < (int)path.size(); t++)
    {
        int from = path[t - 1].location;
        int to = path[t].location;
        if ((int)path_table.table[to].size() > t) // vertex conflicts
        {
            for (auto id : path_table.table[to][t])
            {
                succ = true;
                auto map_key = make_pair(min(agent_id, id), max(agent_id, id));
                if (colliding_pairs.find(map_key) == colliding_pairs.end()) //new collidings
                {
                    colliding_pairs[map_key] = t;
                    if (t <= commit_window) //add to windowed
                    {
                        windowed_colliding_pairs[map_key] = t;
                    }
                }
                else //update
                {
                    int current_time = colliding_pairs[map_key];
                    if(t < current_time) //smaller time, we update
                    {
                        colliding_pairs[map_key] = t;
                        if (t <= commit_window)
                        {
                            windowed_colliding_pairs[map_key] = t;
                        }
                    }
                }
            }
        }
        if (from != to && path_table.table[to].size() >= t && path_table.table[from].size() > t) // edge conflicts
        {
            for (auto a1 : path_table.table[to][t - 1])
            {
                for (auto a2: path_table.table[from][t])
                {
                    if (a1 == a2)
                    {
                        succ = true;
                        auto map_key = make_pair(min(agent_id, a1), max(agent_id, a1));
                        if (colliding_pairs.find(map_key) == colliding_pairs.end()) //new collidings
                        {
                            colliding_pairs[map_key] = t;
                            if (t <= commit_window) //add to windowed
                            {
                                windowed_colliding_pairs[map_key] = t;
                            }
                        }
                        else //update
                        {
                            int current_time = colliding_pairs[map_key];
                            if(t < current_time) //smaller time, we update
                            {
                                colliding_pairs[map_key] = t;
                                if (t <= commit_window)
                                {
                                    windowed_colliding_pairs[map_key] = t;
                                }
                            }
                        }
                        break;
                    }
                }
            }
        }
        //auto id = getAgentWithTarget(to, t);
        //if (id >= 0) // this agent traverses the target of another agent
        //    colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
        if (!path_table.goals.empty() && path_table.goals[to] < t) // target conflicts
        { // this agent traverses the target of another agent
            for (auto id : path_table.table[to][path_table.goals[to]]) // look at all agents at the goal time
            {
                if (agents[id].path.back().location == to) // if agent id's goal is to, then this is the agent we want
                {
                    succ = true;
                    auto map_key = make_pair(min(agent_id, id), max(agent_id, id));
                    if (colliding_pairs.find(map_key) == colliding_pairs.end()) //new collidings
                    {
                        colliding_pairs[map_key] = t;
                        if (t <= commit_window) //add to windowed
                        {
                            windowed_colliding_pairs[map_key] = t;
                        }
                    }
                    else //update
                    {
                        int current_time = colliding_pairs[map_key];
                        if(t < current_time) //smaller time, we update
                        {
                            colliding_pairs[map_key] = t;
                            if (t <= commit_window)
                            {
                                windowed_colliding_pairs[map_key] = t;
                            }
                        }
                    }
                    break;
                }
            }
        }
    }
    int goal = path.back().location; // target conflicts - some other agent traverses the target of this agent
    for (int t = (int)path.size(); t < path_table.table[goal].size(); t++)
    {
        for (auto id : path_table.table[goal][t])
        {
            succ = true;
            auto map_key = make_pair(min(agent_id, id), max(agent_id, id));
            if (colliding_pairs.find(map_key) == colliding_pairs.end()) //new collidings
            {
                colliding_pairs[map_key] = t;
                if (t <= commit_window) //add to windowed
                {
                    windowed_colliding_pairs[map_key] = t;
                }
            }
            else //update
            {
                int current_time = colliding_pairs[map_key];
                if(t < current_time) //smaller time, we update
                {
                    colliding_pairs[map_key] = t;
                    if (t <= commit_window)
                    {
                        windowed_colliding_pairs[map_key] = t;
                    }
                }
            }
        }
    }
    return succ;
}

// void InitLNS::chooseDestroyHeuristicbyALNS()
// {
//     rouletteWheel();
//     switch (selected_neighbor)
//     {
//         case 0 : init_destroy_strategy = TARGET_BASED; break;
//         case 1 : init_destroy_strategy = COLLISION_BASED; break;
//         case 2 : init_destroy_strategy = RANDOM_BASED; break;
//         default : cerr << "ERROR" << endl; exit(-1);
//     }
// }

// bool InitLNS::generateNeighborByEarlyConflict()
// {
//     if (neighbor_size >= agents.size())
//     {
//         neighbor.agents.resize(agents.size());
//         for (int i = 0; i < (int)agents.size(); i++)
//             neighbor.agents[i] = i;
//         return true;
//     }

//     unordered_set<int> neighbors_set;

    //instead of picking a random agent to start with, we have starting point: earliest colliding pairs
    // std::priority_queue<tuple<int,int,int>, vector<tuple<int,int,int>>, CollisionQueueCmp> colliding_pq;
    // //we first need to select top n based on time, so use a pqueue to sort, and track the current largest to filter no need item
    // int max_curr_time = -1;
    // for (auto colliding: total_colliding_pairs)
    // {
    //     //we continue because current time > the existing max time and neighbor size already enough
    //     if (std::get<2>(colliding) > max_curr_time && neighbors_set.size() >= neighbor_size) 
    //         continue;
    //     colliding_pq.push(colliding);
    //     if (std::get<2>(colliding) > max_curr_time)
    //         max_curr_time = std::get<2>(colliding);
    // }
    // while (neighbors_set.size() < neighbor_size && !colliding_pq.empty())
    // {
    //     auto curr = colliding_pq.top();
    //     if(neighbors_set.size() == neighbor_size - 1)
    //     {
    //         if (rand() > 0.5)
    //             neighbors_set.emplace(std::get<1>(curr));
    //         else
    //             neighbors_set.emplace(std::get<0>(curr));
    //     }
    //     else
    //     {
    //         neighbors_set.emplace(std::get<0>(curr));
    //         neighbors_set.emplace(std::get<1>(curr));
    //     }
    //     colliding_pq.pop();
    // }
    // neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
//     if (screen >= 2)
//         cout << "Generate " << neighbor.agents.size() << " neighbors by early conflict" << endl;
//     return true;
// }

bool InitLNS::generateNeighborByEarlyCollisionGraph() //update by chossing the earliest
{
    //unordered_map<int, set<int>> G;
    //unordered_set<int> G;
    unordered_set<int> G;
    auto v = rand()%2 < 1 ? std::get<1>(earlies_colliding_pairs) : std::get<2>(earlies_colliding_pairs);
    findConnectedComponent(time_collision_graph, v, G,neighbor_size);
    assert(G.size() > 1);

    assert(neighbor_size <= (int)agents.size());
    set<int> neighbors_set;
    if ((int)G.size() <= neighbor_size)
    {
        for (const auto& node : G)
            neighbors_set.insert(node);
        int count = 0;
        bool first = true;
        while ((int)neighbors_set.size() < neighbor_size && count < 10)
        {
            int a1;
            if (first)
            {
                a1 = v; //we bias more by chosing the earliest colliding agent
                first = false;
            }
            else
            {
                a1 = *std::next(neighbors_set.begin(), rand() % neighbors_set.size());
            }
            int a2 = randomWalk(a1);
            if (a2 != NO_AGENT)
                neighbors_set.insert(a2);
            else
                count++;
        }
    }
    else
    {
        neighbors_set.insert(v);
        int a = v;
        //std::cout<<"first agent: "<<a<<std::endl;
        //G.erase(v);
        while ((int)neighbors_set.size() < neighbor_size)
        {
            // int a = *std::next(G.begin(), rand() % G.size());
            // neighbors_set.insert(a);
            //int a = *std::next(time_collision_graph[a].begin(), rand() % time_collision_graph[a].size());
            int next_a = a;
            while (neighbors_set.find(next_a) != neighbors_set.end())
            {
                next_a = (*std::next(time_collision_graph[a].begin(), rand() % time_collision_graph[a].size())).first;
            }
            a = next_a;
            //std::cout<<"next agent: "<<a<<" ";
            neighbors_set.insert(a);
        }
        std::cout<<std::endl;
    }
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by collision graph" << endl;
    return true;
}

// bool InitLNS::generateNeighborByTarget()
// {
//     int a = -1;
//     auto r = rand() % (num_of_colliding_pairs * 2);
//     int sum = 0;
//     for (int i = 0 ; i < (int)collision_graph.size(); i++)
//     {
//         sum += (int)collision_graph[i].size();
//         if (r <= sum and !collision_graph[i].empty())
//         {
//             a = i;
//             break;
//         }
//     }
//     assert(a != -1 and !collision_graph[a].empty());
//     set<pair<int,int>> A_start; // an ordered set of (time, id) pair.
//     set<int> A_target;


//     for(int t = 0 ;t< path_table.table[agents[a].path_planner->start_location].size();t++){
//         for(auto id : path_table.table[agents[a].path_planner->start_location][t]){
//             if (id!=a)
//                 A_start.insert(make_pair(t,id));
//         }
//     }



//     agents[a].path_planner->findMinimumSetofColldingTargets(goal_table,A_target);// generate non-wait path and collect A_target


//     if (screen >= 3){
//         cout<<"     Selected a : "<< a<<endl;
//         cout<<"     Select A_start: ";
//         for(auto e: A_start)
//             cout<<"("<<e.first<<","<<e.second<<"), ";
//         cout<<endl;
//         cout<<"     Select A_target: ";
//         for(auto e: A_target)
//             cout<<e<<", ";
//         cout<<endl;
//     }

//     set<int> neighbors_set;

//     neighbors_set.insert(a);

//     if(A_start.size() + A_target.size() >= neighbor_size-1){
//         if (A_start.empty()){
//             vector<int> shuffled_agents;
//             shuffled_agents.assign(A_target.begin(),A_target.end());
//             std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
//             neighbors_set.insert(shuffled_agents.begin(), shuffled_agents.begin() + neighbor_size-1);
//         }
//         else if (A_target.size() >= neighbor_size){
//             vector<int> shuffled_agents;
//             shuffled_agents.assign(A_target.begin(),A_target.end());
//             std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
//             neighbors_set.insert(shuffled_agents.begin(), shuffled_agents.begin() + neighbor_size-2);

//             neighbors_set.insert(A_start.begin()->second);
//         }
//         else{
//             neighbors_set.insert(A_target.begin(), A_target.end());
//             for(auto e : A_start){
//                 //A_start is ordered by time.
//                 if (neighbors_set.size()>= neighbor_size)
//                     break;
//                 neighbors_set.insert(e.second);

//             }
//         }
//     }
//     else if (!A_start.empty() || !A_target.empty()){
//         neighbors_set.insert(A_target.begin(), A_target.end());
//         for(auto e : A_start){
//             neighbors_set.insert(e.second);
//         }

//         set<int> tabu_set;
//         while(neighbors_set.size()<neighbor_size){
//             int rand_int = rand() % neighbors_set.size();
//             auto it = neighbors_set.begin();
//             std::advance(it, rand_int);
//             a = *it;
//             tabu_set.insert(a);

//             if(tabu_set.size() == neighbors_set.size())
//                 break;

//             vector<int> targets;
//             for(auto p: agents[a].path){
//                 if(goal_table[p.location]>-1){
//                     targets.push_back(goal_table[p.location]);
//                 }
//             }

//             if(targets.empty())
//                 continue;
//             rand_int = rand() %targets.size();
//             neighbors_set.insert(*(targets.begin()+rand_int));
//         }
//     }



//     neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
//     if (screen >= 2)
//         cout << "Generate " << neighbor.agents.size() << " neighbors by target" << endl;
//     return true;
// }
// bool InitLNS::generateNeighborRandomly()
// {
//     if (neighbor_size >= agents.size())
//     {
//         neighbor.agents.resize(agents.size());
//         for (int i = 0; i < (int)agents.size(); i++)
//             neighbor.agents[i] = i;
//         return true;
//     }
//     set<int> neighbors_set;
//     auto total = num_of_colliding_pairs * 2 + agents.size();
//     while(neighbors_set.size() < neighbor_size)
//     {
//         vector<int> r(neighbor_size - neighbors_set.size());
//         for (auto i = 0; i < neighbor_size - neighbors_set.size(); i++)
//             r[i] = rand() % total;
//         std::sort(r.begin(), r.end());
//         int sum = 0;
//         for (int i = 0, j = 0; i < agents.size() and j < r.size(); i++)
//         {
//             sum += (int)collision_graph[i].size() + 1;
//             if (sum >= r[j])
//             {
//                 neighbors_set.insert(i);
//                 while (j < r.size() and sum >= r[j])
//                     j++;
//             }
//         }
//     }
//     neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
//     if (screen >= 2)
//         cout << "Generate " << neighbor.agents.size() << " neighbors randomly" << endl;
//     return true;
// }

// Random walk; return the first agent that the agent collides with
int InitLNS::randomWalk(int agent_id)
{
    int t = rand() % agents[agent_id].path.size();
    int loc = agents[agent_id].path[t].location;
    while (t <= path_table.makespan and
           (path_table.table[loc].size() <= t or
           path_table.table[loc][t].empty() or
           (path_table.table[loc][t].size() == 1 and path_table.table[loc][t].front() == agent_id)))
    {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        int step = rand() % next_locs.size();
        auto it = next_locs.begin();
        loc = *std::next(next_locs.begin(), rand() % next_locs.size());
        t = t + 1;
    }
    if (t > path_table.makespan)
        return NO_AGENT;
    else
        return *std::next(path_table.table[loc][t].begin(), rand() % path_table.table[loc][t].size());
}

void InitLNS::writeIterStatsToFile(const string & file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header
    output << //"num of agents," <<
           "sum of costs," <<
           "num of colliding pairs," <<
           "num of windowed collidiing pairs" << "," <<
           "runtime" <<
           endl;

    for (const auto &data : iteration_stats)
    {
        output << //data.num_of_agents << "," <<
               data.sum_of_costs << "," <<
               data.num_of_colliding_pairs << "," <<
               data.num_of_windowed_colliding_pairs << "," <<
               data.runtime <<
               endl;
    }
    output.close();
}

void InitLNS::writeResultToFile(const string & file_name, int sum_of_distances, double preprocessing_time) const
{
    std::ifstream infile(file_name);
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        ofstream addHeads(file_name);
        addHeads << "runtime,num of collisions,solution cost,initial collisions,initial solution cost," <<
                 "sum of distances,iterations,group size," <<
                 "runtime of initial solution,area under curve," <<
                 "LL expanded nodes,LL generated,LL reopened,LL runs," <<
                 "preprocessing runtime,solver name,instance name" << endl;
        addHeads.close();
    }
    uint64_t num_LL_expanded = 0, num_LL_generated = 0, num_LL_reopened = 0, num_LL_runs = 0;
    for (auto & agent : agents)
    {
        agent.path_planner->reset();
        num_LL_expanded += agent.path_planner->accumulated_num_expanded;
        num_LL_generated += agent.path_planner->accumulated_num_generated;
        num_LL_reopened += agent.path_planner->accumulated_num_reopened;
        num_LL_runs += agent.path_planner->num_runs;
    }
    double auc = 0;
    if (!iteration_stats.empty())
    {
        auto prev = iteration_stats.begin();
        auto curr = prev;
        ++curr;
        while (curr != iteration_stats.end() && curr->runtime < time_limit)
        {
            auc += prev->num_of_colliding_pairs * (curr->runtime - prev->runtime);
            prev = curr;
            ++curr;
        }
        auc += prev->num_of_colliding_pairs * (time_limit - prev->runtime);
    }

    ofstream stats(file_name, std::ios::app);
    stats << runtime << "," << iteration_stats.back().num_of_colliding_pairs << "," <<
          sum_of_costs << "," << iteration_stats.front().num_of_colliding_pairs << "," <<
          iteration_stats.front().sum_of_costs << "," << sum_of_distances << "," <<
          iteration_stats.size() << "," << average_group_size << "," <<
          iteration_stats.front().runtime << "," << auc << "," <<
          num_LL_expanded << "," << num_LL_generated << "," << num_LL_reopened << "," << num_LL_runs << "," <<
          preprocessing_time << "," << getSolverName() << "," << instance.getInstanceName() << endl;
    stats.close();
}

void InitLNS::printCollisionGraph() const
{
    cout << "Collision graph: ";
    int edges = 0;
    for (size_t i = 0; i < time_collision_graph.size(); i++)
    {
        for (auto collision : time_collision_graph[i])
        {
            int j = collision.first;
            int t = collision.second;
            if (i < j)
            {
                cout << "(" << i << "," << j <<",t:"<< t << "),";
                edges++;
            }
        }
    }
    cout << endl <<  "|V|=" << time_collision_graph.size() << ", |E|=" << edges << endl;
}


// unordered_map<int, set<int>>& InitLNS::findConnectedComponent(const vector<set<int>>& graph, int vertex,
//                                                                unordered_map<int, set<int>>& sub_graph) //modified due to time collision graph
// {
//     std::queue<int> Q;
//     Q.push(vertex);
//     sub_graph.emplace(vertex, graph[vertex]);
//     while (!Q.empty())
//     {
//         auto v = Q.front(); Q.pop();
//         for (const auto & u : graph[v])
//         {
//             auto ret = sub_graph.emplace(u, graph[u]);
//             if (ret.second) // insert successfully
//                 Q.push(u);
//         }
//     }
//     return sub_graph;
// }

unordered_set<int>& InitLNS::findConnectedComponent(const vector<unordered_map<int,int>>& graph, int vertex,
                                                               unordered_set<int>& candidates,int max_size) //modified due to time collision graph
{
    std::queue<int> Q;
    Q.push(vertex);
    candidates.emplace(vertex);
    while (!Q.empty())
    {
        auto v = Q.front(); Q.pop();
        for (const auto & u : graph[v])
        {
            auto ret = candidates.emplace(u.first);
            if (ret.second) // insert successfully
                Q.push(u.first);
            if (candidates.size() > max_size)
                return candidates;
        }
    }
    return candidates;
}

void InitLNS::printPath() const
{
    for (const auto& agent : agents)
        cout << "Agent " << agent.id << ": " << agent.path << endl;
}

void InitLNS::printResult()
{
    average_group_size = - iteration_stats.front().num_of_agents;
    for (const auto& data : iteration_stats)
        average_group_size += data.num_of_agents;
    if (average_group_size > 0)
        average_group_size /= (double)(iteration_stats.size() - 1);
    assert(!iteration_stats.empty());
    cout << "\t" << getSolverName() << ": "
         << "runtime = " << runtime << ", "
         << "iterations = " << iteration_stats.size() << ", "
         << "colliding pairs = " << num_of_colliding_pairs << ", "
         << "initial colliding pairs = " << iteration_stats.front().num_of_colliding_pairs << ", "
         << "solution cost = " << sum_of_costs << ", "
         << "initial solution cost = " << iteration_stats.front().sum_of_costs << ", "
         << "failed iterations = " << num_of_failures << endl;
    cout << "initial windowed colliding pairs = " << iteration_stats.front().num_of_windowed_colliding_pairs<<","
         << "final windowed colliding pairs = " << num_of_colliding_pairs_windowed<<endl;
    cout << "iterations stats (colliding pairs): "<<endl;
    for (auto iter: iteration_stats)
    {
        cout<<iter.num_of_colliding_pairs<<",";
    } 
    cout <<endl<<"iterations stats (windowed colliding pairs): "<<endl;
    for (auto iter: iteration_stats)
    {
        cout<<iter.num_of_windowed_colliding_pairs<<",";
    } 
    cout <<endl<<"iterations stats (runtime): "<<endl;
    for (auto iter: iteration_stats)
    {
        cout<<iter.runtime<<",";
    } 
    cout <<endl<<"iterations stats (sic): "<<endl;
    for (auto iter: iteration_stats)
    {
        cout<<iter.sum_of_costs<<",";
    } 
    cout<<endl;
}

void InitLNS::clear()
{
    path_table.clear();
    //collision_graph.clear();
    //collision_graph_windowed.clear();
    //total_colliding_pairs.clear();
    time_collision_graph.clear();
    goal_table.clear();
}


bool InitLNS::validatePathTable() const
{
    for (auto i = 0; i < agents.size(); i++)
        assert(path_table.getPath(i) == &agents[i].path);
    return true;
}