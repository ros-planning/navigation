#ifndef _ASTAR_H
#define _ASTAR_H
namespace global_planner {
  int create_nav_plan_astar(const COSTTYPE *costmap, int nx, int ny,
      int* goal, int* start,
      float *plan, int nplan);
      /**
       * @brief  Calculates a plan using the A* heuristic, returns true if one is found
       * @return True if a plan is found, false otherwise
       */
      bool calcNavFnAstar();	/**< calculates a plan, returns true if found */
      /**
       * @brief  Updates the cell at index n using the A* heuristic
       * @param n The index to update
       */
      void updateCellAstar(int n);	/**< updates the cell at index <n>, uses A* heuristic */
            /**
       * @brief  Run propagation for <cycles> iterations, or until start is reached using the best-first A* method with Euclidean distance heuristic
       * @param cycles The maximum number of iterations to run for
       * @return true if the start point is reached
       */
      bool propNavFnAstar(int cycles); /**< returns true if start point found */
}; //end namespace global_planner
#endif
