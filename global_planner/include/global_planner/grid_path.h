#ifndef _GRID_PATH_H
#define _GRID_PATH_H
#include<vector>
#include<global_planner/traceback.h>

namespace global_planner {

    class GridPath : public Traceback{
        public:
            bool getPath(float* potential, int end_x, int end_y, std::vector<std::pair<float, float> >& path);
    };

}; //end namespace global_planner
#endif
