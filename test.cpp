// From Bradley, Hax and Maganti, 'Applied Mathematical Programming', figure 8.1
#include <cstdint>
#include <iostream>
#include<fstream>

#include <map>
#include <vector>
#include <random>
#include"json.hpp"








void test_saving_json(){
    using Node=std::vector<int>;
    std::vector<std::vector<Node>> paths={{{1,2},{1,1}},{{3,4},{3,3}}};
    nlohmann::json my_paths(paths);
    nlohmann::json result;
    result["solution"]=my_paths;
    std::fstream file("./key.json");
    std::cout<<"saving as json"<<std::endl;
    file << result;
}

void test_exmaple_json(){
    nlohmann::json jsonfile;

    jsonfile["foo"] = "bar";

    std::ofstream file("key.json");
    file << jsonfile;
}



int main()
{
    test_saving_json();
    // test_exmaple_json();
   
}