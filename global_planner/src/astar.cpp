#include<global_planner/astar.h>

namespace global_planner {

AStarExpansion::AStarExpansion(int xs, int ys) : Expander(xs, ys) {
}

bool AStarExpansion::calculatePotential(unsigned char* costs, int start_x, int start_y, int end_x, int end_y, int cycles, float* potential){
    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));
    
      std::fill(potential,  potential+ns_,  POT_HIGH);
      potential[start_i] = 0;
      
      
    int goal_i = toIndex(end_x, end_y);
    ROS_INFO("%d %d START", start_x, start_y);
        ROS_INFO("%d %d GOAL", end_x, end_y);
    
    while(queue_.size() > 0){\
        Index top = queue_[0];
        std::pop_heap(queue_.begin(),queue_.end(),greater1());
        queue_.pop_back();
        
        int i = top.i;
        //ROS_INFO("%d %d queue", i%nx_, i/nx_);
        if(i == goal_i)
            return true;/*
        for(int i=0;i<queue_.size();i++){
            ROS_INFO("%f",queue_[i].cost);
            
        }
        ROS_INFO("<<<<<<<<<<<<<<<<");*/
        
        add( costs, potential, potential[i], i+1, end_x, end_y);
        add( costs, potential, potential[i], i-1, end_x, end_y);
        add( costs, potential, potential[i], i+nx_, end_x, end_y);
        add( costs, potential, potential[i], i-nx_, end_x, end_y);
        
        /*        
        
        for(int i=0;i<queue_.size();i++){
            ROS_INFO("%f",queue_[i].cost);
            
        }
        ROS_INFO("===================");*/
    }    
    
    return false;
    
    
    /*
         add current to closedset
         for each neighbor in neighbor_nodes(current)
             tentative_g_score := g_score[current] + dist_between(current,neighbor)
             if neighbor in closedset
                 if tentative_g_score >= g_score[neighbor]
                     continue
 
             if neighbor not in openset or tentative_g_score < g_score[neighbor] 
                 came_from[neighbor] := current
                 g_score[neighbor] := tentative_g_score
                 f_score[neighbor] := g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)
                 if neighbor not in openset
                     add neighbor to openset
 
     return failure
    
    */
}

void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y){
    if(potential[next_i] < POT_HIGH) return;
    
    potential[ next_i ] = prev_potential + costs[next_i] + COST_NEUTRAL;
    int x = next_i % nx_, y = next_i / nx_;
    float distance = abs(end_x - x) + abs(end_y - y);
        
    //ROS_INFO("%d %d | %d %d %f", x,y, end_x, end_y, distance);
    
    queue_.push_back( Index(next_i, potential[ next_i ] + distance * COST_NEUTRAL) );
    //ROS_INFO("%d", queue_.size());
        std::push_heap (queue_.begin(),queue_.end(),greater1());
}
            

}; //end namespace global_planner
