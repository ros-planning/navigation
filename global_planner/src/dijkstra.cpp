#include<global_planner/dijkstra.h>
#include <algorithm>
namespace global_planner {

DijkstraExpansion::DijkstraExpansion(int nx, int ny) :
        Expander(nx, ny), pending_(NULL) {
    // priority buffers
    buffer1_ = new int[PRIORITYBUFSIZE];
    buffer2_ = new int[PRIORITYBUFSIZE];
    buffer3_ = new int[PRIORITYBUFSIZE];

    priorityIncrement_ = 2 * neutral_cost_;
}

//
// Set/Reset map size
//
void DijkstraExpansion::setSize(int xs, int ys) {
    Expander::setSize(xs, ys);
    if (pending_)
        delete[] pending_;

    pending_ = new bool[ns_];
    memset(pending_, 0, ns_ * sizeof(bool));
}

//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool DijkstraExpansion::calculatePotential(unsigned char* costs, int start_x, int start_y, int end_x, int end_y,
                                           int cycles, float* potential) {
    cells_visited_ = 0;
    // priority buffers
    threshold_ = lethal_cost_;
    currentBuffer_ = buffer1_;
    currentEnd_ = 0;
    nextBuffer_ = buffer2_;
    nextEnd_ = 0;
    overBuffer_ = buffer3_;
    overEnd_ = 0;
    memset(pending_, 0, ns_ * sizeof(bool));
    std::fill(potential, potential + ns_, POT_HIGH);

    // set goal
    int k = toIndex(start_x, start_y);
    potential[k] = 0;
    push_cur(k+1);
    push_cur(k-1);
    push_cur(k-nx_);
    push_cur(k+nx_);

    int nwv = 0;			// max priority block size
    int nc = 0;			// number of cells put into priority blocks
    int cycle = 0;		// which cycle we're on

    // set up start cell
    int startCell = toIndex(end_x, end_y);

    for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
            {
        // 
        if (currentEnd_ == 0 && nextEnd_ == 0) // priority blocks empty
            break;

        // stats
        nc += currentEnd_;
        if (currentEnd_ > nwv)
            nwv = currentEnd_;

        // reset pending_ flags on current priority buffer
        int *pb = currentBuffer_;
        int i = currentEnd_;
        while (i-- > 0)
            pending_[*(pb++)] = false;

        // process current priority buffer
        pb = currentBuffer_;
        i = currentEnd_;
        while (i-- > 0)
            updateCell(costs, potential, *pb++);

        // swap priority blocks currentBuffer_ <=> nextBuffer_
        currentEnd_ = nextEnd_;
        nextEnd_ = 0;
        pb = currentBuffer_;		// swap buffers
        currentBuffer_ = nextBuffer_;
        nextBuffer_ = pb;

        // see if we're done with this priority level
        if (currentEnd_ == 0) {
            threshold_ += priorityIncrement_;	// increment priority threshold
            currentEnd_ = overEnd_;	// set current to overflow block
            overEnd_ = 0;
            pb = currentBuffer_;		// swap buffers
            currentBuffer_ = overBuffer_;
            overBuffer_ = pb;
        }

        // check if we've hit the Start cell
        if (potential[startCell] < POT_HIGH)
            break;
    }
    //ROS_INFO("CYCLES %d/%d ", cycle, cycles);
    if (cycle < cycles)
        return true; // finished up here
    else
        return false;
}

//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void DijkstraExpansion::updateCell(unsigned char* costs, float* potential, int n) {
    cells_visited_++;
    // get neighbors
    float u, d, l, r;
    l = potential[n - 1];
    r = potential[n + 1];
    u = potential[n - nx_];
    d = potential[n + nx_];
    //  ROS_INFO("[Update] c: %f  l: %f  r: %f  u: %f  d: %f\n",
    //	 potential[n], l, r, u, d);
    //  ROS_INFO("[Update] cost: %d\n", costs[n]);

    // find lowest, and its lowest neighbor
    float ta, tc;
    if (l < r)
        tc = l;
    else
        tc = r;
    if (u < d)
        ta = u;
    else
        ta = d;

    // do planar wave update
    float c = getCost(costs, n);
    if (c < lethal_cost_)	// don't propagate into obstacles
            {

        float hf = c; // traversability factor
        float dc = tc - ta;		// relative cost between ta,tc
        if (dc < 0) 		// ta is lowest
                {
            dc = -dc;
            ta = tc;
        }

        // calculate new potential
        float pot;
        if (dc >= hf)		// if too large, use ta-only update
            pot = ta + hf;
        else			// two-neighbor interpolation update
        {
            // use quadratic approximation
            // might speed this up through table lookup, but still have to
            //   do the divide
            float d = dc / hf;
            float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
            pot = ta + hf * v;
        }

        //      ROS_INFO("[Update] new pot: %d\n", costs[n]);

        // now add affected neighbors to priority blocks
        if (pot < potential[n]) {
            float le = INVSQRT2 * (float)getCost(costs, n - 1);
            float re = INVSQRT2 * (float)getCost(costs, n + 1);
            float ue = INVSQRT2 * (float)getCost(costs, n - nx_);
            float de = INVSQRT2 * (float)getCost(costs, n + nx_);
            potential[n] = pot;
            //ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]);
            if (pot < threshold_)	// low-cost buffer block
                    {
                if (l > pot + le)
                    push_next(n-1);
                if (r > pot + re)
                    push_next(n+1);
                if (u > pot + ue)
                    push_next(n-nx_);
                if (d > pot + de)
                    push_next(n+nx_);
            } else			// overflow block
            {
                if (l > pot + le)
                    push_over(n-1);
                if (r > pot + re)
                    push_over(n+1);
                if (u > pot + ue)
                    push_over(n-nx_);
                if (d > pot + de)
                    push_over(n+nx_);
            }
        }

    }

}

} //end namespace global_planner
