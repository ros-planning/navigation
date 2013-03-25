#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H

#define PRIORITYBUFSIZE 10000
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <global_planner/planner_core.h>
#include <global_planner/expander.h>

  // inserting onto the priority blocks
#define push_cur(n)  { if (n>=0 && n<ns_ && !pending[n] && costs[n]<lethal_cost_ && curPe<PRIORITYBUFSIZE) { curP[curPe++]=n; pending[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns_ && !pending[n] && costs[n]<lethal_cost_ && nextPe<PRIORITYBUFSIZE){ nextP[nextPe++]=n; pending[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns_ && !pending[n] && costs[n]<lethal_cost_ && overPe<PRIORITYBUFSIZE){ overP[overPe++]=n; pending[n]=true; }}
// potential defs
#define POT_HIGH 1.0e10		// unassigned cell potential


namespace global_planner {
    class DijkstraExpansion : public Expander {
        public:
            DijkstraExpansion(int nx, int ny);
            bool calculatePotential(unsigned char* costs, int start_x, int start_y, int end_x, int end_y, int cycles, float* potential);
                  /**
       * @brief  Sets or resets the size of the map
       * @param nx The x size of the map 
       * @param ny The y size of the map 
       */
      void setSize(int nx, int ny); /**< sets or resets the size of the map */
            
        private:
            
                  /**
       * @brief  Updates the cell at index n  
       * @param n The index to update
       */
      void updateCell(unsigned char* costs, float* potential, int n);	/**< updates the cell at index <n> */
        
        
      /** block priority buffers */
      int *pb1, *pb2, *pb3;		/**< storage buffers for priority blocks */
      int *curP, *nextP, *overP;	/**< priority buffer block ptrs */
      int curPe, nextPe, overPe; /**< end points of arrays */
      bool    *pending;		/**< pending cells during propagation */
      
      /** block priority thresholds */
      float curT;			/**< current threshold */
      float priInc;			/**< priority threshold increment */
      
    };
}; //end namespace global_planner
#endif
