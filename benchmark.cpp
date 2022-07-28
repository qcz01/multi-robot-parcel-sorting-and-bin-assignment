
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
#include "json.hpp"
#include <fstream>
#include "edrp.h"
#include "search_utils.h"
#include "ORCA.hpp"


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

double compute_var(double xx,double x){
    return sqrt(xx-x*x);
}

std::pair<std::string,std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>> get_sorting_center_tasks(Warehouse_Infos &settings){
    std::string name = "warehouse";
    std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>> tgs;
    std::cout<<settings.xmax<<"   "<<settings.ymax<<std::endl;
    Graph graph = get_sorting_warehouse(settings.xmax,settings.ymax);
    //std::cout<<"Sorting graph read! "<<graph.x_size<<" x "<<graph.y_size<<" "<<graph.nodes.size()<<std::endl;
  
    for (size_t i = 10; i <= 100; i += 10) {
        tgs.push_back(std::make_pair(
            i, std::make_pair(get_multi_goal_task_prob(settings,i,100000 * 2 / i, 100000, graph),
                   //get_multi_goal_task(i, 100000 * 2 / i, 100000, graph, 100),
                // get_multi_goal_task(i, 5, 10, graph, 0),
                   graph)));
    }
     return std::make_pair(name, tgs);
}



std::pair<std::string,std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>>> get_sorting_center_tasks_undirected(Warehouse_Infos &settings){
    std::string name = "warehouse";
    std::vector<std::pair<int, std::pair<MultiGoalTask, Graph>>> tgs;
    std::cout<<settings.xmax<<"   "<<settings.ymax<<std::endl;
    Graph graph = get_sorting_warehouse_undirected(settings.xmax,settings.ymax);
    //std::cout<<"Sorting graph read! "<<graph.x_size<<" x "<<graph.y_size<<" "<<graph.nodes.size()<<std::endl;
  
    for (size_t i = 10; i <= 100; i += 10) {
        tgs.push_back(std::make_pair(
            i, std::make_pair(get_multi_goal_task_prob(settings,i,100000 * 2 / i, 100000, graph),
                   //get_multi_goal_task(i, 100000 * 2 / i, 100000, graph, 100),
                // get_multi_goal_task(i, 5, 10, graph, 0),
                   graph)));
    }
     return std::make_pair(name, tgs);
}





std::string result_path = "results/test";



       
/**
 * @brief Get the closest station object
 * 
 * @param settings 
 * @param current 
 * @return int 
 */
int get_closest_station(Warehouse_Infos &settings,const Node &current){
    int closest_index=0;
    double dist_min=9999999;
    for(int i=0;i<settings.station_positions.size();i++){
        auto di=get_manhattan_distance(current,settings.station_positions[i]);
        if(di<dist_min){
            dist_min=di;
            closest_index=i;
        }
    }
    return closest_index;
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
MultiGoalTask get_multi_goal_task_prob(Warehouse_Infos &settings,int num_robots,int num_goals_per_robot,int target_goal_reaching_num, Graph& g){
    std::random_device r;
    std::default_random_engine e1(r());
    size_t num_stations=settings.probability.size();
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> s_gen(0, num_stations-1); // define the range
    std::vector<std::discrete_distribution<int>> random_gen;


    for(int k=0;k<num_stations;k++){
        random_gen.push_back(std::discrete_distribution<int>(settings.probability[k].begin(),settings.probability[k].end()));
    }
    MultiGoalTask t;
    t.num_robots=num_robots;
    t.starts=get_random_nodes(g,num_robots,false,rand());
    t.goals=std::vector<std::vector<Node>>(num_robots,std::vector<Node>());
    t.target_goal_reaching_num=target_goal_reaching_num;
    t.types=std::vector<std::vector<int>>(num_robots,std::vector<int>());
    // std::cout<<"number of goals per robot="<<num_goals_per_robot<<std::endl;
    for(size_t i=0;i<num_robots;i++){
        int k=0;
        auto curr=t.starts[i];
        while(k<num_goals_per_robot){
            // int station=s_gen(gen);
            int station=get_closest_station(settings,curr);
            // std::cout<<curr<<"   "<<station<<"    "<<settings.station_positions[station]<<std::endl;
            int type=random_gen[station](gen);
            // std::cout<<" station position of "<<station<<"  is   "<<settings.station_positions[station]<<std::endl;
            t.goals[i].push_back(settings.station_positions[station]);
            t.types[i].push_back(-1);
            t.types[i].push_back(type);
            t.goals[i].push_back(get_closest_bin(settings.station_positions,settings.bin_assignment,station,type));
            curr=t.goals[i].back();
            // std::cout<<settings.station_positions[station]<<"  to  "<<get_closest_bin(settings.station_positions,settings.bin_assignment,station,type)<<std::endl;
            k+=2;
        }
    }

    // debug the multi-task
    // for(size_t  i=0;i<num_robots;i++){
    //     for(size_t j=0;j<t.goals[i].size();j++){
    //         std::cout<<t.goals[i][j]<<"  ";
    //     }
    //     std::cout<<std::endl;
    // }
    // exit(0);
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



/**
 * @brief 
 * 
 * @param file_name 
 * @param type_bin 
 * @param station_positions 
 * @param xmax 
 * @param ymax 
 */
Warehouse_Infos read_settings_from_json(std::string file_name){
    nlohmann::json data_dict;
    std::ifstream ifs(file_name);
    Warehouse_Infos settings;
    data_dict=nlohmann::json::parse(ifs);
    settings.xmax=data_dict["xmax"];
    settings.ymax=data_dict["ymax"];

    // std::cout<<"map size=  "<<xmax<<" x "<<ymax<<std::endl;

    int num_stations=data_dict["num_stations"];
    
    // std::cout<<"Num of stations= "<<num_stations<<std::endl;
    int num_types=data_dict["num_types"];
    // station_positions.clear();
    std::vector<std::vector<int>> station_data=data_dict["station_positions"];

    for(auto const &n:station_data){
        settings.station_positions.push_back(Node(n[0],n[1]));
    }
  
    nlohmann::json bin_data=data_dict["assignment"];
    // settings.bin_assignment=std::vector<std::vector<Node>>(num_types);
    for(int i=0;i<num_types;i++){
        std::vector<std::vector<int>> bin_i=bin_data[std::to_string(i)];
        for(auto const &n:bin_i){
            settings.bin_assignment[i].push_back(Node(n[0],n[1]));
        }
    }
    std::vector<std::vector<double>> prob=data_dict["probability"];
    settings.probability.swap(prob);
    return settings;
}





std::pair<std::string, std::vector<std::pair<ECBSSolver, std::string>>>
get_ecbs_solvers() {
    auto solvers = std::vector<std::pair<ECBSSolver, std::string>>();
    int i = 1;
    bool finished = false;
    while (!finished) {
        auto solver = ECBSSolver();
        std::string name = "default";
        switch (i) {
            case 0:
                solver.suo = new SUO(0, 0, false);
                name = "Lifelong_ECBS";
                solver.use_suo = false;
                break;
            case 1:
                solver.suo = new SUO(0, 0, false);
                name = "Lifelong_ECBS+Horizon";
                solver.use_suo = false;
                i=3;
                break;
            case 2:
                solver.suo = new SUO(2, 0, false);
                name = "Lifelong_ECBS+VO";
                solver.use_suo = true;
                break;
            case 3:
                solver.suo = new SUO(1, 1, false);
                name = "Lifelong_ECBS+VE";
                solver.use_suo = true;
                break;
            default:
                finished = true;
                break;
        }
       
        if (finished) break;
        solvers.push_back(std::make_pair(solver, name));
        i++;
    }
    return std::make_pair("ecbs_solvers", solvers);
}





std::pair<Graph,MultiGoalTask> generate_instance(Warehouse_Infos & settings,int num_robots){
    Graph graph = get_sorting_warehouse_undirected(settings.xmax,settings.ymax);
    // auto task= get_multi_goal_task(num_robots, 100000 * 2 / num_robots, 100000, graph, 100);
    auto task=get_multi_goal_task_prob(settings,num_robots,5000 * 2 / num_robots, 5000, graph);
    return {graph,task};
}


std::pair<Graph,MultiGoalTask> generate_instance_directed(Warehouse_Infos & settings,int num_robots){
    Graph graph = get_sorting_warehouse(settings.xmax,settings.ymax);
    // auto task= get_multi_goal_task(num_robots, 100000 * 2 / num_robots, 100000, graph, 100);
    auto task=get_multi_goal_task_prob(settings,num_robots,20000 * 2 / num_robots, 20000, graph);
    return {graph,task};
}

using DataToStore=std::vector<std::pair<int, std::vector<std::tuple<double, double, double,double,double,double>>>>;
void save_csv_helper(DataToStore &all_data,std::string solver_name)
{
    result_path = "results/"+solver_name;
    std::experimental::filesystem::create_directory(result_path);
    std::cout<<"saving "<<solver_name<<" result"<<std::endl;
    auto names = std::vector<std::string>();
    names.push_back(solver_name);
    auto file_names = std::vector<std::string>({"computation_time", "computation_time_per_step", "throughput"});
    for (size_t i = 0; i < 3; i++)
    {
        auto dataset =
            std::vector<std::pair<std::string, std::vector<double>>>(3, std::pair<std::string, std::vector<double>>("NA", std::vector<double>(all_data.size(), 0)));
        dataset[0].first = "num_robots";

        for (size_t j = 0; j < names.size(); j++){
            dataset[j + 1].first = names[j];
            dataset[j+2].first="std_err";
        }
            
        for (size_t j = 0; j < all_data.size(); j++)
        {
            dataset[0].second[j] = all_data[j].first;
    
            for (size_t k = 0; k < all_data[j].second.size(); k++)
            {
                if (i == 0){
                    dataset[k + 1].second[j] =std::get<0>(all_data[j].second[k]);
                    dataset[k + 2].second[j] =std::get<1>(all_data[j].second[k]);
                }
                
                if (i == 1){
                    dataset[k + 1].second[j] =std::get<2>(all_data[j].second[k]);
                    dataset[k + 2].second[j] =std::get<3>(all_data[j].second[k]);
                }
                if (i == 2)
                    dataset[k + 1].second[j] =std::get<4>(all_data[j].second[k]);
                    dataset[k + 2].second[j] =std::get<5>(all_data[j].second[k]);
            }
        }
        std::string fname = result_path + "/" + "warehouse" + "-" +
                            std::to_string(300) + "-" + solver_name +
                            "-" + file_names[i];
        std::cout<<"writing csv to  "<<fname<<std::endl;
        write_csv(fname, dataset);
    }
}

void test_multi_goal_ddm(std::string file_name)
{
    Warehouse_Infos settings = read_settings_from_json(file_name);
    size_t ddm_horizon = 300;
    auto all_data =DataToStore();
    load_database(); // Load database file.
    for (int num_robots = 10; num_robots <= 100; num_robots += 10)
    {
        auto pack_data = std::make_pair(num_robots, std::vector<std::tuple<double, double, double,double,double,double>>());
        double sum_comp_time = 0, sum_comp_time2=0, sum_comp_time_per_step = 0, sum_comp_time_per_step2=0, sum_throughput = 0, sum_throughput2=0;
        int num_trials = 10;
        for (int k = 0; k < num_trials; k++)
        {
            auto graph_task = generate_instance(settings, num_robots);
            auto start_time = std::chrono::high_resolution_clock::now();
            auto solver = DdmSolver();

            auto paths = solver.solve(graph_task.second, graph_task.first, ddm_horizon);
            double time_cost = time_elapsed(start_time);
            sum_comp_time += time_cost;
            sum_comp_time2+=time_cost*time_cost;
            sum_comp_time_per_step +=time_cost /(ddm_horizon);
            sum_comp_time_per_step2+=pow( time_cost / ddm_horizon,2);
            sum_throughput +=graph_task.second.target_goal_reaching_num / double(paths.size() - 1);
            sum_throughput2+=pow(graph_task.second.target_goal_reaching_num/double(paths.size() - 1),2);
        }
        std::cout<<"Average comp_time per step  "<<sum_comp_time_per_step / num_trials<<"  average throughput  "<<sum_throughput/num_trials<<std::endl;
        pack_data.second.push_back({sum_comp_time / num_trials,compute_var(sum_comp_time2/num_trials,sum_comp_time/num_trials),
                                    sum_comp_time_per_step / num_trials,compute_var(sum_comp_time_per_step2/num_trials,sum_comp_time_per_step/num_trials),
                                    sum_throughput / num_trials,compute_var(sum_throughput2/num_trials,sum_throughput/num_trials)});
        all_data.push_back(pack_data);
    }
    save_csv_helper(all_data,"ddm");
}

void test_multi_goal_drp(std::string file_name)
{
    Warehouse_Infos settings = read_settings_from_json(file_name);
    size_t ddm_horizon = 300;
    auto all_data =DataToStore();
    // load_database(); // Load database file.
    for (int num_robots = 500; num_robots <= 500; num_robots += 30)
    {
        auto pack_data = std::make_pair(num_robots, std::vector<std::tuple<double, double, double,double,double,double>>());
        double sum_comp_time = 0, sum_comp_time2=0, sum_comp_time_per_step = 0, sum_comp_time_per_step2=0, sum_throughput = 0, sum_throughput2=0;
        int num_trials = 1;
        for (int k = 0; k < num_trials; k++)
        {
            auto graph_task = generate_instance_directed(settings, num_robots);
            auto start_time = std::chrono::high_resolution_clock::now();
            auto solver = DRP();

            auto paths = solver.solve(graph_task.second, graph_task.first, ddm_horizon);
            

            // save_result_as_json(graph_task.second,"./sol500.json",paths);
            double time_cost = time_elapsed(start_time);
            sum_comp_time += time_cost;
            sum_comp_time2+=time_cost*time_cost;
            sum_comp_time_per_step +=
                time_cost /(ddm_horizon);
            sum_comp_time_per_step2+=pow( time_cost /  ddm_horizon,2);
            sum_throughput +=graph_task.second.target_goal_reaching_num / double(paths.size() - 1);
            sum_throughput2+=pow(graph_task.second.target_goal_reaching_num/double(paths.size() - 1),2);
        }
        std::cout<<"Average comp_time per step  "<<sum_comp_time_per_step / num_trials<<"  average throughput  "<<sum_throughput/num_trials<<std::endl;
        pack_data.second.push_back({sum_comp_time / num_trials,compute_var(sum_comp_time2/num_trials,sum_comp_time/num_trials),
                                    sum_comp_time_per_step / num_trials,compute_var(sum_comp_time_per_step2/num_trials,sum_comp_time_per_step/num_trials),
                                    sum_throughput / num_trials,compute_var(sum_throughput2/num_trials,sum_throughput/num_trials)});
        all_data.push_back(pack_data);
    }
    // save_csv_helper(all_data,"drp");
}


void test_multi_goal_edrp(std::string file_name)
{
    Warehouse_Infos settings = read_settings_from_json(file_name);
    size_t ddm_horizon = 300;
    auto all_data =DataToStore();
    // load_database(); // Load database file.
    for (int num_robots = 500; num_robots <= 500; num_robots += 30)
    {
        auto pack_data = std::make_pair(num_robots, std::vector<std::tuple<double, double, double,double,double,double>>());
        double sum_comp_time = 0, sum_comp_time2=0, sum_comp_time_per_step = 0, sum_comp_time_per_step2=0, sum_throughput = 0, sum_throughput2=0;
        int num_trials = 1;
        for (int k = 0; k < num_trials; k++)
        {
            auto graph_task = generate_instance_directed(settings, num_robots);
            auto start_time = std::chrono::high_resolution_clock::now();
            auto solver = EDRP();

            auto paths = solver.solve(graph_task.second, graph_task.first, ddm_horizon);
            save_result_as_json(graph_task.second,"./esol500.json",paths);
            double time_cost = time_elapsed(start_time);
            sum_comp_time += time_cost;
            sum_comp_time2+=time_cost*time_cost;
            sum_comp_time_per_step +=
                time_cost /(ddm_horizon);
            sum_comp_time_per_step2+=pow( time_cost /  ddm_horizon,2);
            sum_throughput +=graph_task.second.target_goal_reaching_num / double(paths.size() - 1);
            sum_throughput2+=pow(graph_task.second.target_goal_reaching_num/double(paths.size() - 1),2);
        }
        std::cout<<"Average comp_time per step  "<<sum_comp_time_per_step / num_trials<<"  average throughput  "<<sum_throughput/num_trials<<std::endl;
        pack_data.second.push_back({sum_comp_time / num_trials,compute_var(sum_comp_time2/num_trials,sum_comp_time/num_trials),
                                    sum_comp_time_per_step / num_trials,compute_var(sum_comp_time_per_step2/num_trials,sum_comp_time_per_step/num_trials),
                                    sum_throughput / num_trials,compute_var(sum_throughput2/num_trials,sum_throughput/num_trials)});
        all_data.push_back(pack_data);
    }
    save_csv_helper(all_data,"edrp");
}

void test_multi_goal_ecbs(std::string file_name)
{
    Warehouse_Infos settings = read_settings_from_json(file_name);
    size_t ddm_horizon = 600;
    auto all_data =DataToStore();
    
    for (int num_robots = 30; num_robots <= 300; num_robots += 30)
    {
        auto pack_data = std::make_pair(num_robots, std::vector<std::tuple<double, double, double,double,double,double>>());
        double sum_comp_time = 0, sum_comp_time2=0, sum_comp_time_per_step = 0, sum_comp_time_per_step2=0, sum_throughput = 0, sum_throughput2=0;
        int num_trials = 10;
        for (int k = 0; k < num_trials; k++)
        {
            auto graph_task = generate_instance(settings, num_robots);
            auto start_time = std::chrono::high_resolution_clock::now();
            auto solver = ECBSSolver();
            solver.suo = new SUO(0, 0, false);
            
            solver.use_suo = false;
            // (
            //     task, graph, solvers[s].first.default_horizon,
            //     solvers[s].first.default_horizon, 1.5)
            auto paths = solver.solve(graph_task.second, graph_task.first,solver.default_horizon,solver.default_horizon,1.5);
            double time_cost = time_elapsed(start_time);
            sum_comp_time += time_cost;
            sum_comp_time2+=time_cost*time_cost;
            double makespan= double(paths.size() - 1);
            sum_comp_time_per_step +=
                time_cost / makespan;
            sum_comp_time_per_step2+=pow( time_cost / ddm_horizon,2);
            // std::cout<<makespan<<std::endl;
            sum_throughput +=graph_task.second.target_goal_reaching_num / double(paths.size() - 1);
            sum_throughput2+=pow(graph_task.second.target_goal_reaching_num/double(paths.size() - 1),2);
        }
    
        std::cout<<"number of robots="<<num_robots<< "Average comp_time per step  "<<sum_comp_time_per_step / num_trials<<"  average throughput  "<<sum_throughput/num_trials<<std::endl;
        pack_data.second.push_back({sum_comp_time / num_trials,compute_var(sum_comp_time2/num_trials,sum_comp_time/num_trials),
                                    sum_comp_time_per_step / num_trials,compute_var(sum_comp_time_per_step2/num_trials,sum_comp_time_per_step/num_trials),
                                    sum_throughput / num_trials,compute_var(sum_throughput2/num_trials,sum_throughput/num_trials)});
        all_data.push_back(pack_data);
    }
    save_csv_helper(all_data,"ecbs");
}


void test_multi_goal_ecbs_directed(std::string file_name)
{
    Warehouse_Infos settings = read_settings_from_json(file_name);
    size_t ddm_horizon = 300;
    auto all_data =DataToStore();
    
    for (int num_robots = 30; num_robots <= 300; num_robots += 30)
    {
        auto pack_data = std::make_pair(num_robots, std::vector<std::tuple<double, double, double,double,double,double>>());
        double sum_comp_time = 0, sum_comp_time2=0, sum_comp_time_per_step = 0, sum_comp_time_per_step2=0, sum_throughput = 0, sum_throughput2=0;
        int num_trials = 10;
        for (int k = 0; k < num_trials; k++)
        {
            auto graph_task = generate_instance_directed(settings, num_robots);
            auto start_time = std::chrono::high_resolution_clock::now();
            auto solver = ECBSSolver();
            solver.suo = new SUO(0, 0, false);
            
            solver.use_suo = false;
            // (
            //     task, graph, solvers[s].first.default_horizon,
            //     solvers[s].first.default_horizon, 1.5)
            auto paths = solver.solve(graph_task.second, graph_task.first,solver.default_horizon,solver.default_horizon,1.5);
            double time_cost = time_elapsed(start_time);
            sum_comp_time += time_cost;
            sum_comp_time2+=time_cost*time_cost;
            double makespan=double(paths.size() - 1);
            sum_comp_time_per_step +=
                time_cost / makespan;
            sum_comp_time_per_step2+=pow( time_cost / makespan,2);
            sum_throughput +=graph_task.second.target_goal_reaching_num / double(paths.size() - 1);
            sum_throughput2+=pow(graph_task.second.target_goal_reaching_num/double(paths.size() - 1),2);
        }
        std::cout<<"Average comp_time per step  "<<sum_comp_time_per_step / num_trials<<"  average throughput  "<<sum_throughput/num_trials<<std::endl;
        pack_data.second.push_back({sum_comp_time / num_trials,compute_var(sum_comp_time2/num_trials,sum_comp_time/num_trials),
                                    sum_comp_time_per_step / num_trials,compute_var(sum_comp_time_per_step2/num_trials,sum_comp_time_per_step/num_trials),
                                    sum_throughput / num_trials,compute_var(sum_throughput2/num_trials,sum_throughput/num_trials)});
        all_data.push_back(pack_data);
    }
    save_csv_helper(all_data,"ecbsd");
}


void save_result_as_json(MultiGoalTask &task,std::string file_name,std::vector<std::vector<Node>>&paths){
 
    nlohmann::json data_json;
    using Location=std::vector<int>;
    std::vector<std::vector<Location>> all_tasks(task.num_robots,std::vector<Location>());
    for(int i=0;i<task.num_robots;i++){
        for(int k=0;k<2*task.finished_goals[i];k++){
            all_tasks[i].push_back({task.goals[i][k].x,task.goals[i][k].y,task.types[i][k]});
        }
    }
    nlohmann::json task_json(all_tasks);
    std::cout<<task_json.size()<<std::endl;
    data_json["tasks"]=task_json;
    std::vector<Location> starts;
    for(auto &start:task.starts){
     
        starts.push_back({start.x,start.y});
    }
    nlohmann::json starts_json(starts);
    data_json["starts"]=starts_json;


    std::vector<std::vector<Location>> paths_data(task.num_robots,std::vector<Location>());
    for(int i=0;i<task.num_robots;i++){
        for(int t=0;t<paths.size();t++){
            auto v=paths[t][i];
            paths_data[i].push_back({v.x,v.y});
        }
        // for(auto const &v:paths[t]){
        //     paths_data[t].push_back({v.x,v.y});
        // }
    }
    nlohmann::json paths_json(paths_data);
    data_json["paths"]=paths_json;
    std::ofstream file;
    file.open(file_name);
    std::cout<<data_json["tasks"].size()<<std::endl;
    std::cout<<"save as  "<<file_name<<std::endl;
    file<< data_json<<std::endl; 
    // file<<"what is wrong?"<<std::endl;


}



void test_ORCA(std::string file_name){
    Warehouse_Infos settings = read_settings_from_json(file_name);
    size_t ddm_horizon = 600;
    auto all_data =DataToStore();
    int num_trials=10;
    for (int num_robots = 10; num_robots <= 100; num_robots += 10)
    {
        auto pack_data = std::make_pair(num_robots, std::vector<std::tuple<double, double, double,double,double,double>>());
        double sum_comp_time = 0, sum_comp_time2=0, sum_comp_time_per_step = 0, sum_comp_time_per_step2=0, sum_throughput = 0, sum_throughput2=0;
        for (int k = 0; k < num_trials; k++)
        {
            auto graph_task = generate_instance(settings, num_robots);
            ORCA orca_solver;
            auto start_time = std::chrono::high_resolution_clock::now();
            orca_solver.solve(graph_task.second,graph_task.first,ddm_horizon);
            double time_cost = time_elapsed(start_time);
            sum_comp_time += time_cost;
            sum_comp_time2+=time_cost*time_cost;
            double makespan=ddm_horizon;
            sum_comp_time_per_step +=
                time_cost / makespan;
            sum_comp_time_per_step2+=pow( time_cost / makespan,2);
            // std::cout<<"throguhput="<<graph_task.second.target_goal_reaching_num/makespan<<std::endl;
            sum_throughput +=graph_task.second.target_goal_reaching_num / makespan;
            sum_throughput2+=pow(graph_task.second.target_goal_reaching_num/makespan,2);
        }
        std::cout<<"Average comp_time per step  "<<sum_comp_time_per_step / num_trials<<"  average throughput  "<<sum_throughput/num_trials<<std::endl;
        pack_data.second.push_back({sum_comp_time / num_trials,compute_var(sum_comp_time2/num_trials,sum_comp_time/num_trials),
                                    sum_comp_time_per_step / num_trials,compute_var(sum_comp_time_per_step2/num_trials,sum_comp_time_per_step/num_trials),
                                    sum_throughput / num_trials,compute_var(sum_throughput2/num_trials,sum_throughput/num_trials)});
        all_data.push_back(pack_data);

    }

    save_csv_helper(all_data,"orca");
}