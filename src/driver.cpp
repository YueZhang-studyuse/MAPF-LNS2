#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "LNS.h"
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "PIBT/pibt.h"

//initial commit


/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
        ("output,o", po::value<string>(), "output file name (no extension)")
        ("inputPaths", po::value<string>(), "input file for paths")
        ("outputPaths", po::value<string>(), "output file for paths")
        ("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(0),
		        "screen option (0: none; 1: LNS results; 2:LNS detailed results; 3: MAPF detailed results)")
        ("commitStep,c", po::value<int>()->default_value(1), "steps per commit")
        ("wait", po::value<int>()->default_value(0), "wait time")

		("stats", po::value<string>(), "output stats file")

		// solver
		("solver", po::value<string>()->default_value("LNS"), "solver (LNS, A-BCBS, A-EECBS)")
		("sipp", po::value<bool>()->default_value(true), "Use SIPP as the single-agent solver")
		("seed", po::value<int>()->default_value(0), "Random seed")

        // params for LNS
        ("initLNS", po::value<bool>()->default_value(true),
             "use LNS to find initial solutions if the initial sovler fails")
        ("neighborSize", po::value<int>()->default_value(8), "Size of the neighborhood")
        ("maxIterations", po::value<int>()->default_value(0), "maximum number of iterations")
        ("initAlgo", po::value<string>()->default_value("PP"),
                "MAPF algorithm for finding the initial solution (EECBS, PP, PPS, CBS, PIBT, winPIBT)")
        ("replanAlgo", po::value<string>()->default_value("PP"),
                "MAPF algorithm for replanning (EECBS, CBS, PP)")
        ("destoryStrategy", po::value<string>()->default_value("Adaptive"),
                "Heuristics for finding subgroups (Random, RandomWalk, Intersection, Adaptive)")
        ("pibtWindow", po::value<int>()->default_value(5),
             "window size for winPIBT")
        ("winPibtSoftmode", po::value<bool>()->default_value(true),
             "winPIBT soft mode")
        ("truncatePaths", po::value<bool>()->default_value(true),
             "Truncate initial paths to maximize agents at goals")

         // params for initLNS
         ("initDestoryStrategy", po::value<string>()->default_value("Adaptive"),
          "Heuristics for finding subgroups (Target, Collision, Random, Adaptive)")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

    PIBTPPS_option pipp_option;
    pipp_option.windowSize = vm["pibtWindow"].as<int>();
    pipp_option.winPIBTSoft = vm["winPibtSoftmode"].as<bool>();

    po::notify(vm);

	srand((int)time(0));

	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
		vm["agentNum"].as<int>());
    double time_limit = vm["cutoffTime"].as<double>();
    int screen = vm["screen"].as<int>();
	srand(vm["seed"].as<int>());

	if (vm["solver"].as<string>() == "LNS")
    {
        LNS lns(instance, time_limit,
                vm["initAlgo"].as<string>(),
                vm["replanAlgo"].as<string>(),
                vm["destoryStrategy"].as<string>(),
                vm["neighborSize"].as<int>(),
                //vm["maxIterations"].as<int>(),
                MAX_TIMESTEP,
                vm["initLNS"].as<bool>(),
                vm["initDestoryStrategy"].as<string>(),
                vm["sipp"].as<bool>(),
                vm["truncatePaths"].as<bool>(),
                screen, pipp_option);
        if (vm.count("inputPaths") and !lns.loadPaths(vm["inputPaths"].as<string>()))
        {
            cerr << "The input path file " << vm["inputPaths"].as<string>() << " does not exist" << endl;
            exit(-1);
        }
        bool succ = lns.run();
        if (succ)
        {
            lns.validateSolution();
            if (vm.count("outputPaths"))
                lns.writePathsToFile(vm["outputPaths"].as<string>());
        }
        if (vm.count("output"))
            lns.writeResultToFile(vm["output"].as<string>());
        if (vm.count("stats"))
            lns.writeIterStatsToFile(vm["stats"].as<string>());
        // lns.writePathsToFile("path.txt");
    }
    else if (vm["solver"].as<string>() == "A-BCBS") // anytime BCBS(w, 1)
    {
        AnytimeBCBS bcbs(instance, time_limit, screen);
        bcbs.run();
        bcbs.validateSolution();
        if (vm.count("output"))
            bcbs.writeResultToFile(vm["output"].as<string>() + ".csv");
        if (vm.count("stats"))
            bcbs.writeIterStatsToFile(vm["stats"].as<string>());
    }
    else if (vm["solver"].as<string>() == "A-EECBS") // anytime EECBS
    {
        AnytimeEECBS eecbs(instance, time_limit, screen);
        eecbs.run();
        eecbs.validateSolution();
        if (vm.count("output"))
            eecbs.writeResultToFile(vm["output"].as<string>() + ".csv");
        if (vm.count("stats"))
            eecbs.writeIterStatsToFile(vm["stats"].as<string>());
    }
    else if (vm["solver"].as<string>() == "LNS-Online") // my commit lns
    {
        double initial_time = 300;
        double step_time = 1;
        int commit_step = vm["commitStep"].as<int>();

        int wait = vm["wait"].as<int>();

        int max_iterations = MAX_TIMESTEP;
        //note here I put future path start = commited path (I assume start at timestep 0?)
        vector<list<int>> commited_paths;
        vector<list<int>> future_paths;
        commited_paths.resize(instance.getDefaultNumberOfAgents());
        future_paths.resize(instance.getDefaultNumberOfAgents());
        bool commited_done = false;
        bool initial_run = true;
        bool initial_succ = false;
        double total_step = 0;
        list<int> solution_costs;
        list<int> iterations;
        list<int> success_iterations;
        list<int> ll_searchs;
        list<double> ll_times;

        double last_improve_pers = -1;

        LNS lns(instance, initial_time,
                vm["initAlgo"].as<string>(),
                vm["replanAlgo"].as<string>(),
                vm["destoryStrategy"].as<string>(),
                vm["neighborSize"].as<int>(),
                MAX_TIMESTEP,
                vm["initLNS"].as<bool>(),
                vm["initDestoryStrategy"].as<string>(),
                vm["sipp"].as<bool>(),
                vm["truncatePaths"].as<bool>(),
                screen, pipp_option);
        while(!commited_done)
        {

            //update start locations
            //for (auto path: commited_paths)
            for (int i = 0; i < instance.getDefaultNumberOfAgents();i++)
            {
                if (commited_paths[i].size()>0) 
                {
                    //change instance.start locations according commited_paths
                    instance.setStart(i,commited_paths[i].back());
                }
                //else we do nothing, just keep as in instances
            }
            //init lns
            bool succ;
            if (initial_run)
            {
                //run lns to get next commit
                //lns.setIterations(0);
                //lns.setRuntimeLimit(step_time + wait);
                lns.setRuntimeLimit(1+wait); //set initial run to step time as default
                lns.setRuntimeLimit(20);
                succ = lns.run();
                if (succ)
                {
                    lns.validateSolution();

                    // double success = lns.iteration_stats.size();
                    // if (success > 1)
                    // {
                    //     auto iter_per_s = (success/commit_step)/(lns.runtime - lns.initial_solution_runtime);
                    //     commit_step = 1;
                    //     //test
                    //     double avg_path = lns.sum_of_costs/(double)instance.getDefaultNumberOfAgents();
                    //     double esti_search = iter_per_s*8*commit_step/(1-commit_step/avg_path);
                    //     while (esti_search > 0 && esti_search <instance.getDefaultNumberOfAgents())
                    //     {
                    //         //cout<<esti_search<<endl;
                    //         commit_step++;
                    //         esti_search = iter_per_s*8*commit_step/(1-commit_step/avg_path);
                    //     }
                    //     //ceil(((double)instance.getDefaultNumberOfAgents()/8)/success);
                    // }
                    // cout<<commit_step<<" "<<success<<endl;

                    //try add commit anyway
                    commit_step++;
                    last_improve_pers = (lns.initial_sum_of_costs - lns.sum_of_costs)/lns.(runtime-initial_solution_runtime);

                    lns.commitPath(commit_step,commited_paths,future_paths,false,total_step);
                    solution_costs.emplace_back(lns.sum_of_costs + instance.getDefaultNumberOfAgents()*wait);
                    iterations.emplace_back(lns.iteration_stats.size());
                    success_iterations.emplace_back(lns.iteration_stats.size() - lns.num_of_failures);
                    ll_searchs.emplace_back(lns.num_ll_search);
                    ll_times.emplace_back(lns.sum_ll_time);
                    total_step+=commit_step;
                    std::cout<<"Preprocessing time: "<<lns.preprocessing_time<<std::endl;
                    initial_succ = true;
                }
                else
                {
                    initial_succ = false;
                    cout<<"not success"<<endl;
                    //if failed, commit++
                    commit_step++;

                    lns.commitPath(commit_step,commited_paths,future_paths,false,total_step);
                    solution_costs.emplace_back(lns.sum_of_costs + instance.getDefaultNumberOfAgents()*wait);
                    iterations.emplace_back(lns.iteration_stats.size());
                    success_iterations.emplace_back(lns.iteration_stats.size() - lns.num_of_failures);
                    ll_searchs.emplace_back(lns.num_ll_search);
                    ll_times.emplace_back(lns.sum_ll_time);
                    total_step+=commit_step;
                    std::cout<<"Preprocessing time: "<<lns.preprocessing_time<<std::endl;
                }
                lns.writePathsToFile("InitialPath.txt");
                initial_run = false;
            }
            else
            {
                lns.clearAll(vm["destoryStrategy"].as<string>());
                lns.setIterations(max_iterations);
                lns.setRuntimeLimit(step_time*commit_step);
                
                if (initial_succ)
                {
                    //load initial path
                    if (!lns.loadPaths(future_paths))
                    {
                        cerr << "The input path wrong" << endl;
                        exit(-1);
                    }
                }
                else
                {
                    lns.has_initial_solution = false;
                }
                succ = lns.run();
                initial_succ = succ;
                if (succ)
                {
                    lns.validateSolution();
                }
                future_paths.clear();
                future_paths.resize(instance.getDefaultNumberOfAgents());
                if (!initial_succ)
                {
                    //need more time to commit
                    commit_step++;
                }
                // else
                // {
                //     //dynamic commit
                //     //first get improve per sec last time
                //     //then improve per sec this time
                //     double this_improve_pers = (lns.initial_sum_of_costs - lns.sum_of_costs)/lns.(runtime-initial_solution_runtime);
                    
                // }

                    // double success = lns.iteration_stats.size();
                    // if (success > 1)
                    // {
                        
                    //     auto iter_per_s = success/commit_step;
                    //     commit_step = 1;
                    //     //test
                    //     double avg_path = lns.sum_of_costs/(double)instance.getDefaultNumberOfAgents();
                    //     double esti_search = iter_per_s*8*commit_step/(1-commit_step/avg_path);
                    //     while (esti_search > 0 && esti_search <instance.getDefaultNumberOfAgents())
                    //     {
                    //         commit_step++;
                    //         esti_search = iter_per_s*8*commit_step/(1-commit_step/avg_path);
                    //     }
                    //     //ceil(((double)instance.getDefaultNumberOfAgents()/8)/success);
                    // }
                    // cout<<commit_step<<" "<<success<<endl;

                    lns.commitPath(commit_step,commited_paths,future_paths,true,total_step);
                    int sic = 0;
                    for (int i = 0; i < instance.getDefaultNumberOfAgents();i++)
                    {
                        auto path = commited_paths[i];
                        sic+= path.size()-1;
                        if (path.back() == instance.getGoals()[i] && future_paths[i].size() == 1)
                        {
                            //bool first = true;
                            for (auto it = path.rbegin(); it != path.rend();++it)
                            {
                                // if (first)
                                // {
                                //     first = false;
                                //     continue;
                                // }
                                if (*it == instance.getGoals()[i])
                                {
                                    sic--;
                                }
                                else
                                {
                                    break;
                                }
                            }
                            sic++;
                        }
                        sic+=future_paths[i].size()-1;
                        // if (sic > solution_costs.back())
                        // {
                        //     cout<<"wrong"<<endl;
                        // }
                    }
                    solution_costs.emplace_back(sic + instance.getDefaultNumberOfAgents()*wait);
                    iterations.emplace_back(lns.iteration_stats.size());
                    success_iterations.emplace_back(lns.iteration_stats.size() - lns.num_of_failures);
                    ll_searchs.emplace_back(lns.num_ll_search);
                    ll_times.emplace_back(lns.sum_ll_time);
                    total_step+=commit_step;
                    std::cout<<"Preprocessing time: "<<lns.preprocessing_time<<std::endl;
            }


            //check if commit is done
            if (!commited_paths[0].empty())
            {
                for (auto path: future_paths)
                {
                    if (path.size() <= 1) //first path is the start location
                        commited_done = true;
                    else
                    {
                        commited_done = false;
                        break;
                    }    
                }

            }
        }
        //maybe add a validation to see if solution is correct
        // LNS lns(instance, step_time*commit_step,
        //         vm["initAlgo"].as<string>(),
        //         vm["replanAlgo"].as<string>(),
        //         vm["destoryStrategy"].as<string>(),
        //         vm["neighborSize"].as<int>(),
        //         max_iterations,
        //         vm["initLNS"].as<bool>(),
        //         vm["initDestoryStrategy"].as<string>(),
        //         vm["sipp"].as<bool>(),
        //         vm["truncatePaths"].as<bool>(),
        //         screen, pipp_option);
        //lns.setIterations(max_iterations);
        lns.validateCommitSolution(commited_paths);
        cout<<"num of commits per step:"<<endl;
        cout<<commit_step<<endl;
        cout<<"time per step:"<<endl;
        cout<<step_time<<endl;
        cout<<"sic in iterations:"<<endl;
        cout<<"makespan: "<<endl;
        cout<<total_step<<endl;
        for(auto sic: solution_costs)
        {
            cout<<sic<<" ";
        }
        cout<<endl;
        cout<<"num of iterations:"<<endl;
        for(auto i:iterations)
        {
            cout<<i<<" ";
        }
        cout<<endl;
        cout<<"num of success iterations:"<<endl;
        for(auto i:success_iterations)
        {
            cout<<i<<" ";
        }
        cout<<endl;

        cout<<"num of low level searches:"<<endl;
        for(auto i:ll_searchs)
        {
            cout<<i<<" ";
        }
        cout<<endl;

         cout<<"sum of low level searches times:"<<endl;
        for(auto i:ll_times)
        {
            cout<<i<<" ";
        }
        cout<<endl;

        cout<<"commit paths:"<<endl;
        for (int i = 0; i < instance.getDefaultNumberOfAgents();i++)
        {
            for (auto vertex: commited_paths[i])
            {
                cout<<vertex<<", ";
            }
            cout<<endl;
        }
        cout<<endl;

        string name = vm["output"].as<string>();
        std::ifstream infile(name);
        bool exist = infile.good();
        infile.close();
        if (!exist)
        {
            ofstream addHeads(name);
            addHeads << "num of agents," <<
            "sum of costs," <<
            "initial sum of costs," <<
            "wait," << endl;
            addHeads.close();
        }
        
        ofstream stats(name, std::ios::app);
        stats << instance.getDefaultNumberOfAgents() << "," <<
                solution_costs.back() << "," <<
                solution_costs.front() << "," <<
                wait << "," << endl;
        stats.close();
    }
    else
    {
	    cerr << "Solver " << vm["solver"].as<string>() << " does not exist!" << endl;
	    exit(-1);
    }
	return 0;

}