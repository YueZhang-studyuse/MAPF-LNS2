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
        double commit_step = vm["commitStep"].as<int>();
        int max_iterations = MAX_TIMESTEP;
        //note here I put future path start = commited path (I assume start at timestep 0?)
        vector<list<int>> commited_paths;
        vector<list<int>> future_paths;
        commited_paths.resize(instance.getDefaultNumberOfAgents());
        future_paths.resize(instance.getDefaultNumberOfAgents());
        bool commited_done = false;
        bool initial_run = true;
        double total_step = 0;
        list<int> solution_costs;
        LNS lns(instance, initial_time,
                vm["initAlgo"].as<string>(),
                vm["replanAlgo"].as<string>(),
                vm["destoryStrategy"].as<string>(),
                vm["neighborSize"].as<int>(),
                0,
                vm["initLNS"].as<bool>(),
                vm["initDestoryStrategy"].as<string>(),
                vm["sipp"].as<bool>(),
                vm["truncatePaths"].as<bool>(),
                screen, pipp_option);
        lns.commit_window = commit_step;
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
                lns.setRuntimeLimit(10);
                succ = lns.run();
                if (succ)
                {
                    lns.validateSolution();
                    lns.commitPath(commit_step,commited_paths,future_paths,false,total_step);
                    solution_costs.emplace_back(lns.sum_of_costs);
                    total_step+=commit_step;
                    std::cout<<"Preprocessing time: "<<lns.preprocessing_time<<std::endl;
                }
                else
                {
                    cerr << "Initialise solution failed" << endl;
                    exit(-1);
                }
                lns.writePathsToFile("InitialPath.txt");
                initial_run = false;
                return 0;
            }
            else
            {
                lns.clearAll(vm["destoryStrategy"].as<string>());
                lns.setIterations(max_iterations);
                lns.setRuntimeLimit(step_time*commit_step);
                
                //load initial path
                if (!lns.loadPaths(future_paths))
                {
                    cerr << "The input path wrong" << endl;
                    exit(-1);
                }
                succ = lns.run();
                if (succ)
                {
                    lns.validateSolution();
                    future_paths.clear();
                    future_paths.resize(instance.getDefaultNumberOfAgents());
                    lns.commitPath(commit_step,commited_paths,future_paths,true,total_step);
                    int sic = 0;
                    for (int i = 0; i < instance.getDefaultNumberOfAgents();i++)
                    {
                        auto path = commited_paths[i];
                        sic+= path.size()-1;
                        if (path.back() == instance.getGoals()[i])
                        {
                            bool first = true;
                            for (auto it = path.rbegin(); it != path.rend();++it)
                            {
                                if (first)
                                {
                                    first = false;
                                    continue;
                                }
                                if (*it == instance.getGoals()[i])
                                {
                                    sic--;
                                }
                            }
                        }
                        sic+=future_paths[i].size()-1;
                        if (sic > solution_costs.back())
                        {
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
                            cout<<"future paths"<<endl;
                            for (int i = 0; i < instance.getDefaultNumberOfAgents();i++)
                            {
                                for (auto vertex: future_paths[i])
                                {
                                    cout<<vertex<<", ";
                                }
                                cout<<endl;
                            }
                            cout<<endl;
                        }
                    }
                    solution_costs.emplace_back(sic);
                    total_step+=commit_step;
                    std::cout<<"Preprocessing time: "<<lns.preprocessing_time<<std::endl;
                }
                else
                {
                    cerr << "Iteration failed" << endl;
                    exit(-1);
                }
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
        lns.validateCommitSolution(commited_paths);
        cout<<"num of commits per step:"<<endl;
        cout<<commit_step<<endl;
        cout<<"time per step:"<<endl;
        cout<<step_time<<endl;
        cout<<"sic in iterations:"<<endl;
        for(auto sic: solution_costs)
        {
            cout<<sic<<" ";
        }
        cout<<endl;
    }
    else if (vm["solver"].as<string>() == "LNS2-Online") // my commit lns
    {
        double step_time = 1;
        double commit_step = vm["commitStep"].as<int>();
        double initial_time = step_time*commit_step;
        int max_iterations = MAX_TIMESTEP;
        //note here I put future path start = commited path (I assume start at timestep 0?)
        vector<list<int>> commited_paths;
        vector<list<int>> future_paths;
        commited_paths.resize(instance.getDefaultNumberOfAgents());
        future_paths.resize(instance.getDefaultNumberOfAgents());
        bool commited_done = false;
        bool initial_run = true;
        bool solution_feasible = false;
        bool conflict_in_window = false;
        double total_step = 0;
        list<int> solution_costs;
        LNS lns(instance, initial_time,
                vm["initAlgo"].as<string>(),
                vm["replanAlgo"].as<string>(),
                vm["destoryStrategy"].as<string>(),
                vm["neighborSize"].as<int>(),
                max_iterations,
                vm["initLNS"].as<bool>(),
                vm["initDestoryStrategy"].as<string>(),
                vm["sipp"].as<bool>(),
                vm["truncatePaths"].as<bool>(),
                screen, pipp_option);
        lns.commit_window = commit_step;
        lns.setRuntimeLimit(step_time*commit_step+lns.preprocessing_time);
        while(!commited_done)
        {
            cout<<"step: "<<total_step<<endl;
            //update start locations
            for (int i = 0; i < instance.getDefaultNumberOfAgents();i++)
            {
                if (commited_paths[i].size()>0) 
                {
                    //change instance.start locations according commited_paths
                    instance.setStart(i,commited_paths[i].back());
                }
                //else we do nothing, just keep as in instances
            }
            //we break up into two situations: 
            //1. solution still infeasible (use lns2), 2. solution feasible (use lns)
            bool succ;
            if (initial_run || !solution_feasible)
            {
                std::cout<<"run lns2 commit"<<std::endl;
                if (!initial_run)
                {
                    lns.clearAll(vm["destoryStrategy"].as<string>());
                    if (!lns.loadPaths(future_paths))
                    {
                        cerr << "The input path wrong" << endl;
                        exit(-1);
                    }
                    lns.setRuntimeLimit(step_time*commit_step);
                    lns.setIterations(max_iterations);
                }
                //run lns to get next commit
                succ = lns.runLns2(!initial_run,conflict_in_window); 
                // lns.validateSolution();
                if (succ)
                {
                    //lns.validateSolutionByWindow(commit_step);
                    //lns.validateSolutionDebugMode();
                    future_paths.clear();
                    future_paths.resize(instance.getDefaultNumberOfAgents());
                    if (initial_run)
                    {
                        lns.commitPath(commit_step,commited_paths,future_paths,false,total_step);
                        solution_costs.emplace_back(lns.sum_of_costs);
                    }
                    else
                    {
                        lns.commitPath(commit_step,commited_paths,future_paths,true,total_step);
                        int sic = 0;
                        for (int i = 0; i < instance.getDefaultNumberOfAgents();i++)
                        {
                            auto path = commited_paths[i];
                            sic+= path.size()-1;
                            if (path.back() == instance.getGoals()[i])
                            {
                                bool first = true;
                                for (auto it = path.rbegin(); it != path.rend();++it)
                                {
                                    if (first)
                                    {
                                        first = false;
                                        continue;
                                    }
                                    if (*it == instance.getGoals()[i])
                                    {
                                        sic--;
                                    }
                                }
                            }
                            sic+=future_paths[i].size()-1;
                        }
                        solution_costs.emplace_back(sic);
                    }
                    
                    if (lns.lns2_solutin_conflicts == 0)
                        solution_feasible = true;
                    total_step+=commit_step;
                    //lns.validateCommitSolution(commited_paths);
                }
                else
                {
                    cerr << "Initialise LNS2 failed" << endl;
                    exit(-1);
                }

                lns.writePathsToFile("InitialPath.txt");
                initial_run = false;
            }
            else
            {
                lns.clearAll(vm["destoryStrategy"].as<string>());
                lns.setIterations(max_iterations);
                lns.setRuntimeLimit(step_time*commit_step);
                //load initial path
                if (!lns.loadPaths(future_paths))
                {
                    cerr << "The input path wrong" << endl;
                    exit(-1);
                }
                succ = lns.run();
                if (succ)
                {
                    //lns.validateSolutionByWindow(commit_step);
                    future_paths.clear();
                    future_paths.resize(instance.getDefaultNumberOfAgents());
                    lns.commitPath(commit_step,commited_paths,future_paths,true,total_step);
                    int sic = 0;
                    for (int i = 0; i < instance.getDefaultNumberOfAgents();i++)
                    {
                        auto path = commited_paths[i];
                        sic+= path.size()-1;
                        if (path.back() == instance.getGoals()[i])
                        {
                            bool first = true;
                            for (auto it = path.rbegin(); it != path.rend();++it)
                            {
                                if (first)
                                {
                                    first = false;
                                    continue;
                                }
                                if (*it == instance.getGoals()[i])
                                {
                                    sic--;
                                }
                            }
                        }
                        sic+=future_paths[i].size()-1;
                        if (sic > solution_costs.back())
                        {
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
                            cout<<"future paths"<<endl;
                            for (int i = 0; i < instance.getDefaultNumberOfAgents();i++)
                            {
                                for (auto vertex: future_paths[i])
                                {
                                    cout<<vertex<<", ";
                                }
                                cout<<endl;
                            }
                            cout<<endl;
                        }
                    }
                    solution_costs.emplace_back(sic);
                    total_step+=commit_step;
                }
                else
                {
                    cerr << "Iteration failed" << endl;
                    exit(-1);
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
        }
        // cout<<"commited path: "<<endl;;
        // for (int i = 0; i < commited_paths.size(); i++)
        // {
        //     cout<<"agent "<<i<<" ";
        //     for (auto p: commited_paths[i])
        //     {
        //         cout<<p<<",";
        //     }
        //     cout<<endl;
        // }
        //cout<<endl;
        lns.validateCommitSolution(commited_paths);
        cout<<"num of commits per step:"<<endl;
        cout<<commit_step<<endl;
        cout<<"time per step:"<<endl;
        cout<<step_time<<endl;
        cout<<"sic in iterations:"<<endl;
        for(auto sic: solution_costs)
        {
            cout<<sic<<" ";
        }
        cout<<endl;
    }
	else
    {
	    cerr << "Solver " << vm["solver"].as<string>() << " does not exist!" << endl;
	    exit(-1);
    }
	return 0;

}