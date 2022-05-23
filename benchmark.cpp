
/**
 * @file benchmark.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include"benchmark.h"
#include"drp.h"
#include <random>

/**
 * @brief Construct a new std::pair<std::string,std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>>get warehouse multi goal tasks small object
 * 
 */
std::pair<std::string,std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>>get_warehouse_multi_goal_tasks_small() {
    std::string name = "warehouse";
    std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>> tgs;
    Graph graph = get_sorting_warehouse();
    std::cout<<"Sorting graph read! "<<graph.x_size<<" x "<<graph.y_size<<" "<<graph.nodes.size()<<std::endl;
  
    for (size_t i = 10; i <= 100; i += 10) {
        tgs.push_back(std::make_pair(
            i, std::make_pair(
                   get_multi_goal_task(i, 100000 * 2 / i, 100000, graph, 100),
                // get_multi_goal_task(i, 5, 10, graph, 0),
                   graph)));
    }
    return std::make_pair(name, tgs);
}

/**
 * @brief 
 * 
 */
void test_multi_goal_drp() {
  
    auto tfs = std::vector<MultiTaskFunction>(
        {get_warehouse_multi_goal_tasks_small});
    // auto sfs = std::vector<ECBSSolverFunction>({get_ecbs_solvers});
    auto solvers = std::vector<std::pair<DRP, std::string>>();
    auto solver=new DRP();
    for (auto tf : tfs){
       multi_goal_test_helper_drp(tf,0);

    }
}



std::string result_path = "results/test";
/**
 * @brief 
 * 
 * @param tf 
 * @param horizon 
 */
void multi_goal_test_helper_drp(MultiTaskFunction tf, int horizon){
    std::experimental::filesystem::create_directory(result_path);
    size_t drp_horizon=300;
    auto all_data = std::vector<
        std::pair<int, std::vector<std::tuple<double, double, double>>>>();
    // Get tasks
    std::pair<std::string,
              std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>>
        mt = tf();
    std::string task_name = mt.first;
    auto& tasks = mt.second;
    // Get solvers
    // std::pair<std::string, std::vector<std::pair<DdmSolver, std::string>>> ns =
    //     sf();
    // std::string solver_set = ns.first;
    std::string solver_set = "DRP_solvers";
    auto drp_solver= DRP();
    std::vector<std::pair<DRP, std::string>> solvers ={{drp_solver,"drp_lifelong"}};
    auto names = std::vector<std::string>();
    names.push_back("drp_lifelong");
    // for (size_t s = 0; s < solvers.size(); s++) {
    //     names.push_back(solvers[s].second);
    // }
    // Run tests
    std::cout << "Running: Task: " << task_name
              << ", solver set: " << solver_set << std::endl;
    std::cout<<"Num of Tasks= "<<tasks.size()<<std::endl;
    std::cout<<"Solvers.size="<<solvers.size()<<std::endl;
    for (auto task_pack : tasks) {
        auto computation_time = std::vector<double>(solvers.size(), 0);
        auto computation_time_per_step = std::vector<double>(solvers.size(), 0);
        auto throughput = std::vector<double>(solvers.size(), 0);
        std::cout << "n = " << task_pack.first << std::endl;
        auto& task_graph = task_pack.second;
        auto& task = task_graph.first;
        auto& graph = task_graph.second;
        for (size_t s = 0; s < solvers.size(); s++) {
            auto start_time = std::chrono::high_resolution_clock::now();
            auto paths = solvers[s].first.solve(
                task, graph, drp_horizon);
            double time_cost = time_elapsed(start_time);
            computation_time[s] = time_cost;
            computation_time_per_step[s] =
                time_cost /
                (double(paths.size() - 1) / drp_horizon);
            throughput[s] =
                task.target_goal_reaching_num / double(paths.size() - 1);
            std::cout<<"makespan="<<paths.size()<<std::endl;
            assert(paths.size()>=1);
            std::cout<<"troughput "<<throughput[s]<<" reached goals="<<task.target_goal_reaching_num<<std::endl;
        }
        // Final count
        auto pack_data = std::make_pair(
            task_pack.first, std::vector<std::tuple<double, double, double>>());
        for (size_t s = 0; s < solvers.size(); s++) {
            auto unit_data = std::tuple<double, double, double>(
                computation_time[s], computation_time_per_step[s],
                throughput[s]);
            pack_data.second.push_back(unit_data);
        }
        all_data.push_back(pack_data);
        // Output result
        std::cout << "Generating plots..." << std::endl;
        auto file_names = std::vector<std::string>(
            {"computation_time", "computation_time_per_step", "throughput"});
        for (size_t i = 0; i < 3; i++) {
            auto dataset =
                std::vector<std::pair<std::string, std::vector<double>>>(
                    solvers.size() + 1,
                    std::pair<std::string, std::vector<double>>(
                        "NA", std::vector<double>(all_data.size(), 0)));
            dataset[0].first = "num_robots";
            for (size_t j = 0; j < names.size(); j++)
                dataset[j + 1].first = names[j];
            for (size_t j = 0; j < all_data.size(); j++) {
                dataset[0].second[j] = all_data[j].first;
                for (size_t k = 0; k < all_data[j].second.size(); k++) {
                    if (i == 0)
                        dataset[k + 1].second[j] =
                            std::get<0>(all_data[j].second[k]);
                    if (i == 1)
                        dataset[k + 1].second[j] =
                            std::get<1>(all_data[j].second[k]);
                    if (i == 2)
                        dataset[k + 1].second[j] =
                            std::get<2>(all_data[j].second[k]);
                }
            }
            std::string fname = result_path + "/" + task_name + "-" +
                                std::to_string(horizon) + "-" + solver_set +
                                "-" + file_names[i];
            write_csv(fname, dataset);
            std::cout<<"PLoting csv filename="<<(fname + ".csv")<<std::endl;
            system(
                std::string("python3 make_plot_multi_goal.py " + fname + ".csv")
                    .c_str());
        }
    }
    // Clean up
    // for (size_t s = 0; s < solvers.size(); s++) {
    //     delete solvers[s].first.suo;
    // }                                       
}


/**
 * @brief Get the multi goal task object
 * 
 * @param station_positions 
 * @param probability 
 * @param num_robots 
 * @param num_goals_per_robot 
 * @param target_goal_reaching_num 
 * @param g 
 * @param seed 
 * @return MultiGoalTask 
 */
MultiGoalTask get_multi_goal_task_(std::vector<Node> &station_positions,std::vector<std::vector<double>>&probability,std::map<int,std::vector<Node>>&type_bin,int num_robots,int num_goals_per_robot,int target_goal_reaching_num, Graph& g){
    std::random_device r;
    std::default_random_engine e1(r());
    size_t num_stations=probability.size();
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> s_gen(0, num_stations); // define the range
    std::vector<std::discrete_distribution<int>> random_gen;


    for(int k=0;k<num_stations;k++){
        random_gen.push_back(std::discrete_distribution<int>(probability[k].begin(),probability[k].end()));
    }
    MultiGoalTask t;
    t.num_robots=num_robots;
    t.starts=get_random_nodes(g,num_robots,false,rand());
    t.goals=std::vector<std::vector<Node>>();
    t.target_goal_reaching_num=target_goal_reaching_num;
    for(size_t i=0;i<num_robots;i++){
        int k=0;
        while(k<num_goals_per_robot){
            int station=s_gen(gen);
            int type=random_gen[station](gen);
            t.goals[i].push_back(station_positions[station]);
            t.goals[i].push_back(get_closest_bin(station_positions,type_bin,station,type));
            k+=2;
        }
    }
    return t;
}



/**
 * @brief Get the closest bin object
 * 
 * @param station_position 
 * @param bin_map 
 * @param station 
 * @param type 
 * @return Node 
 */
Node get_closest_bin(std::vector<Node> &station_position,std::map<int,std::vector<Node>> &bin_map,int station,int type){
    int min_dist=100000;
    Node u;
    for(int i=0;i<bin_map[type].size();i++){
        auto bin=bin_map[type][i];
        std::vector<Node> possible_destinations;
        possible_destinations.push_back(Node(bin.x+1,bin.y));
        possible_destinations.push_back(Node(bin.x-1,bin.y));
        possible_destinations.push_back(Node(bin.x,bin.y+1));
        possible_destinations.push_back(Node(bin.x,bin.y-1));
        for(auto &nbr:possible_destinations){
            auto dist=get_manhattan_distance(nbr,station_position[station]);
            if(dist<min_dist){
                min_dist=dist;
                u=nbr;
            }
        }
    }
    return u;
}