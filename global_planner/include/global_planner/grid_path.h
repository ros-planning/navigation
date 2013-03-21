#ifndef _GRID_PATH_H
#define _GRID_PATH_H
#include<vector>
namespace global_planner {

    class GridPath {
        public:
            bool getPath(float* potential, int end_x, int end_y, std::vector<std::pair<float, float> >& path);
            void setSize(int xs, int ys){ xs_ = xs; ys_ = ys; }
            inline int getIndex(int x, int y){ return x + y * xs_; } 
        private:
            int xs_, ys_;
    };

}; //end namespace global_planner
#endif
