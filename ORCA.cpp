/**
 * @file ORCA.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include"ORCA.hpp"


void ORCA::solve(MultiGoalTask &t,Graph &g,int horizon){
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();
    setupScenario(sim,t,g);
    while(true){
        timeCounter++;
        if(timeCounter*dt>horizon){
            t.target_goal_reaching_num=0;
            for(size_t i=0;i<goal_ids.size();i++){
                t.target_goal_reaching_num+=goal_ids[i];
            }
            // std::cout<<"total number tasks"<<t.target_goal_reaching_num<<" throughout="<<(float)t.target_goal_reaching_num/horizon<<std::endl;
            break;
        }
        for(int i=0;i<t.num_robots;i++){
            if(reachedGoal(sim,i)==true){
                
                //update the goal
                goal_ids[i]++;
                auto gi=t.goals[i][goal_ids[i]];
                goals[i]=RVO::Vector2(gi.x,gi.y);
            }
        }
     
        setPreferredVelocities(sim);
        sim->doStep();
        // std::cout<<"current time counter="<<timeCounter<<std::endl;
    }
    // std::cout<<"Num of agents="<<static_cast<int>(sim->getNumAgents())<<std::endl;
    // int totalTasks=0;
    // for(int i=0;i<t.num_robots;i++){
    //     totalTasks+=goal_ids[i];
    // }
    // std::cout<<"throughput= "<<(float)totalTasks/horizon<<std::endl;
    delete sim;
}

void ORCA::setPreferredVelocities(RVO::RVOSimulator *sim){
    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
		RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		}

		sim->setAgentPrefVelocity(i, goalVector);

		/*
		 * Perturb a little to avoid deadlocks due to perfect symmetry.
		 */
		float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		float dist = std::rand() * 0.0001f / RAND_MAX;

		sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +
		                          dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
	}
}

void ORCA::setupScenario(RVO::RVOSimulator *sim,MultiGoalTask &t,Graph &g){
    
    float neighborDist=15.0f;
    size_t maxNeighbors=10;
    float timeHorizon=5.0f;
    float timeHorizonObst=5.0f;
    float radius=0.3;
    float maxSpeed=1;
    sim->setTimeStep(dt);
    sim->setAgentDefaults(neighborDist,maxNeighbors,timeHorizon,timeHorizonObst,radius,maxSpeed);
    // sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);
    addRobots(sim,t);
    addObstacles(sim,g);
}


void ORCA::addObstacles(RVO::RVOSimulator *sim,Graph &g){
    for(auto const &obs: g.obstacles){
        std::vector<RVO::Vector2> obstacle;
        obstacle.push_back(RVO::Vector2(obs.x-0.5f,obs.y-0.5f));
        obstacle.push_back(RVO::Vector2(obs.x+0.5f,obs.y-0.5f));
        obstacle.push_back(RVO::Vector2(obs.x+0.5f,obs.y+0.5f));
        obstacle.push_back(RVO::Vector2(obs.x-0.5f,obs.y+0.5f));
        sim->addObstacle(obstacle);
    }
    sim->processObstacles();

}   


void ORCA::addRobots(RVO::RVOSimulator *sim,MultiGoalTask &t){
    goal_ids=std::vector<size_t>(t.num_robots,0);
    // std::cout<<"t.num_robots="<<t.num_robots<<"   "<<goal_ids.size()<<std::endl;
    for(auto const &start:t.starts){
        sim->addAgent(RVO::Vector2(start.x,start.y));
    }
    int num_robots=t.num_robots;
    goals=std::vector<RVO::Vector2>(num_robots);
    for(int i=0;i<num_robots;i++){
        auto gi=t.goals[i][goal_ids[i]];
        goals[i]=RVO::Vector2(gi.x,gi.y);
    }
}

bool ORCA::reachedGoal(RVO::RVOSimulator *sim,int agent){
    if(RVO::absSq(sim->getAgentPosition(agent)-goals[agent])>0.01f*0.01f) return false;
    return true;
}