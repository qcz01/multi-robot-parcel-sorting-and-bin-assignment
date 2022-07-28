#include "benchmark.h"


int main(){
    test_multi_goal_drp("./python/genetic.json");
 
    // test_multi_goal_ddm("./python/genetic.json");
    // test_multi_goal_ecbs("./python/huge_genetic.json");
    // test_multi_goal_edrp("./python/large_genetic.json");
    // debug("./python/large_genetic.json");
    // test_ORCA("./python/mip.json");
    return 0;
}