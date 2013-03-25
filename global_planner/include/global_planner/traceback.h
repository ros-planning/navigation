#ifndef _TRACEBACK_H
#define _TRACEBACK_H
#include<vector>
namespace global_planner {

    class Traceback {
        public:
            virtual bool getPath(float* potential, int end_x, int end_y, std::vector<std::pair<float, float> >& path) = 0;
            virtual void setSize(int xs, int ys){ xs_ = xs; ys_ = ys; }
            inline int getIndex(int x, int y){ return x + y * xs_; } 
        protected:
            int xs_, ys_;
    };

}; //end namespace global_planner
#endif
