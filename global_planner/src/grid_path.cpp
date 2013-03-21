
#include <global_planner/grid_path.h>
#include <algorithm>
#include <stdio.h>
namespace global_planner {

bool GridPath::getPath(float* potential, int end_x, int end_y, std::vector<std::pair<float, float> >& path){
    std::pair<int, int> current;
    current.first = end_x;
    current.second = end_y;
    
    path.push_back(current);
    
    while(potential[getIndex(current.first, current.second)] > 0){
        float min_val = 1e10;
        int min_x = 0, min_y = 0;
        for(int xd = -1; xd <= 1; xd++){
            for(int yd = -1; yd <= 1; yd++){
                if(xd==0 && yd==0)
                    continue;
                int x = current.first + xd, y = current.second + yd;
                int index = getIndex(x,y);
                if( potential[index] < min_val){
                    min_val = potential[index];
                    min_x = x;
                    min_y = y;
                }
            }
        }
        if(min_x == 0 && min_y==0)
            return false;
        current.first = min_x;
        current.second= min_y;
        path.push_back(current);
        
    }
    //std::reverse(path.begin(), path.end());
    return true;
}


}; //end namespace global_planner

