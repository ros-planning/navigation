#ifndef _ASTAR_H
#define _ASTAR_H

#include <global_planner/planner_core.h>
#include <global_planner/expander.h>
#include <vector>
#include <algorithm>

namespace global_planner {
    class Index {
        public:
            Index(int a, float b){ i = a; cost = b; }
            int i;
            float cost;
    };
    
    struct greater1{
      bool operator()(const Index& a,const Index& b) const{
        return a.cost>b.cost;
      }
    };


    class AStarExpansion : public Expander {
        public:
            AStarExpansion(int nx, int ny);
            bool calculatePotential(unsigned char* costs, int start_x, int start_y, int end_x, int end_y, int cycles, float* potential);
        private:
            void add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y);
            std::vector<Index> queue_; 
    };

}; //end namespace global_planner
#endif


      /**
       * @brief  Updates the cell at index n  
       * @param n The index to update
       */
      //void updateCell(unsigned char* costs, float* potential, int n);	/**< updates the cell at index <n> */

