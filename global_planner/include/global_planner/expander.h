#ifndef _EXPANDER_H
#define _EXPANDER_H
namespace global_planner {

    class Expander {
        public:
            Expander(int nx, int ny) : unknown_(true), lethal_cost_(254), neutral_cost_(50) { setSize(nx, ny); }
            virtual bool calculatePotential(unsigned char* costs, int start_x, int start_y, int end_x, int end_y, int cycles, float* potential) = 0; 
            
            /**
             * @brief  Sets or resets the size of the map
             * @param nx The x size of the map 
             * @param ny The y size of the map 
             */
            virtual void setSize(int nx, int ny){ nx_ = nx; ny_ = ny; ns_ = nx * ny; } /**< sets or resets the size of the map */
            void setLethalCost(unsigned char lethal_cost){ lethal_cost_ = lethal_cost; }
            void setHasUnknown(bool unknown){ unknown_ = unknown; }
      
        protected:
            inline int toIndex(int x, int y) { return x + nx_*y; }
      
            int nx_, ny_, ns_;		/**< size of grid, in pixels */
            bool unknown_;
            unsigned char lethal_cost_, neutral_cost_;
            int cells_visited_;
    };
    
}; //end namespace global_planner
#endif
