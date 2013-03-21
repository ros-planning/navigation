#include<global_planner/dijkstra.h>
#include <algorithm>
namespace global_planner {

DijkstraExpansion::DijkstraExpansion(int nx, int ny)
{
    // create cell arrays
    pending = NULL;
    setSize(nx,ny);

    // priority buffers
    pb1 = new int[PRIORITYBUFSIZE];
    pb2 = new int[PRIORITYBUFSIZE];
    pb3 = new int[PRIORITYBUFSIZE];
    COST_OBS = 254;
    COST_NEUTRAL = 50;

    // for Dijkstra (breadth-first), set to COST_NEUTRAL
    // for A* (best-first), set to COST_NEUTRAL
    priInc = 2*COST_NEUTRAL;	
    
}

//
// Set/Reset map size
//
void DijkstraExpansion::setSize(int xs, int ys)
{
      nx = xs;
      ny = ys;
      ns = nx*ny;

      if(pending)
        delete[] pending;

      pending = new bool[ns];
      memset(pending,   0,        ns*sizeof(bool));
}


  //
  // main propagation function
  // Dijkstra method, breadth-first
  // runs for a specified number of cycles,
  //   or until it runs out of cells to update,
  //   or until the Start cell is found (atStart = true)
  //

bool DijkstraExpansion::calculatePotential(unsigned char* costs, int start_x, int start_y, int end_x, int end_y, int cycles, float* potential)
{
          // priority buffers
      curT = COST_OBS;
      curP = pb1; 
      curPe = 0;
      nextP = pb2;
      nextPe = 0;
      overP = pb3;
      overPe = 0;
      memset(pending,   0,        ns*sizeof(bool));
      std::fill(potential,  potential+ns,  POT_HIGH);

      // set goal
      int k = toIndex(start_x, start_y);
      potential[k] = 0;
      push_cur(k+1);
      push_cur(k-1);
      push_cur(k-nx);
      push_cur(k+nx);      

      int nwv = 0;			// max priority block size
      int nc = 0;			// number of cells put into priority blocks
      int cycle = 0;		// which cycle we're on

      // set up start cell
      int startCell = toIndex(end_x, end_y);

      for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
      {
        // 
        if (curPe == 0 && nextPe == 0) // priority blocks empty
          break;

        // stats
        nc += curPe;
        if (curPe > nwv)
          nwv = curPe;

        // reset pending flags on current priority buffer
        int *pb = curP;
        int i = curPe;			
        while (i-- > 0)		
          pending[*(pb++)] = false;

        // process current priority buffer
        pb = curP; 
        i = curPe;
        while (i-- > 0)		
          updateCell(costs, potential, *pb++);

        // swap priority blocks curP <=> nextP
        curPe = nextPe;
        nextPe = 0;
        pb = curP;		// swap buffers
        curP = nextP;
        nextP = pb;

        // see if we're done with this priority level
        if (curPe == 0)
        {
          curT += priInc;	// increment priority threshold
          curPe = overPe;	// set current to overflow block
          overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

        // check if we've hit the Start cell
        if (potential[startCell] < POT_HIGH)
            break;
      }
      //ROS_INFO("CYCLES %d/%d ", cycle, cycles);
      if (cycle < cycles) return true; // finished up here
      else return false;
    }



  // 
  // Critical function: calculate updated potential value of a cell,
  //   given its neighbors' values
  // Planar-wave update calculation from two lowest neighbors in a 4-grid
  // Quadratic approximation to the interpolated value 
  // No checking of bounds here, this function should be fast
  //

#define INVSQRT2 0.707106781

  inline void
    DijkstraExpansion::updateCell(unsigned char* costs, float* potential, int n)
    {
      // get neighbors
      float u,d,l,r;
      l = potential[n-1];
      r = potential[n+1];		
      u = potential[n-nx];
      d = potential[n+nx];
      //  ROS_INFO("[Update] c: %f  l: %f  r: %f  u: %f  d: %f\n", 
      //	 potential[n], l, r, u, d);
      //  ROS_INFO("[Update] cost: %d\n", costs[n]);
      

      // find lowest, and its lowest neighbor
      float ta, tc;
      if (l<r) tc=l; else tc=r;
      if (u<d) ta=u; else ta=d;

      // do planar wave update
      if (costs[n] < COST_OBS)	// don't propagate into obstacles
      {
        float hf = (float)costs[n]+ COST_NEUTRAL; // traversability factor
        float dc = tc-ta;		// relative cost between ta,tc
        if (dc < 0) 		// ta is lowest
        {
          dc = -dc;
          ta = tc;
        }

        // calculate new potential
        float pot;
        if (dc >= hf)		// if too large, use ta-only update
          pot = ta+hf;
        else			// two-neighbor interpolation update
        {
          // use quadratic approximation
          // might speed this up through table lookup, but still have to 
          //   do the divide
          float d = dc/hf;
          float v = -0.2301*d*d + 0.5307*d + 0.7040;
          pot = ta + hf*v;
        }

        //      ROS_INFO("[Update] new pot: %d\n", costs[n]);

        // now add affected neighbors to priority blocks
        if (pot < potential[n])
        {
          float le = INVSQRT2*(float)costs[n-1];
          float re = INVSQRT2*(float)costs[n+1];
          float ue = INVSQRT2*(float)costs[n-nx];
          float de = INVSQRT2*(float)costs[n+nx];
          potential[n] = pot;
          //ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]);
          if (pot < curT)	// low-cost buffer block 
          {
            if (l > pot+le) push_next(n-1);
            if (r > pot+re) push_next(n+1);
            if (u > pot+ue) push_next(n-nx);
            if (d > pot+de) push_next(n+nx);
          }
          else			// overflow block
          {
            if (l > pot+le) push_over(n-1);
            if (r > pot+re) push_over(n+1);
            if (u > pot+ue) push_over(n-nx);
            if (d > pot+de) push_over(n+nx);
          }
        }

      }

    }


}; //end namespace global_planner
