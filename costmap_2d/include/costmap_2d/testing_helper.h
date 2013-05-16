#ifndef TESTING_HELPER_H
#define TESTING_HELPER_H

#include<costmap_2d/cost_values.h>
#include<costmap_2d/costmap_2d.h>

const double MAX_Z(1.0);

void setValues( costmap_2d::Costmap2D& costmap, const unsigned char* map )
{
  int index=0;
  for(int i=0;i<costmap.getSizeInCellsY();i++){
    for(int j=0;j<costmap.getSizeInCellsX();j++){
      costmap.setCost(j,i,map[index]);
    }
  }
}

char printableCost( unsigned char cost )
{
  switch( cost )
  {
  case costmap_2d::NO_INFORMATION: return '?';
  case costmap_2d::LETHAL_OBSTACLE: return 'L';
  case costmap_2d::INSCRIBED_INFLATED_OBSTACLE: return 'I';
  case costmap_2d::FREE_SPACE: return '.';
  default: return '0' + (unsigned char) (10 * cost / 255);
  }
}

void printMap( costmap_2d::Costmap2D& costmap )
{
  printf("map:\n");
  for(int i=0;i<costmap.getSizeInCellsY();i++){
    for(int j=0;j<costmap.getSizeInCellsX();j++){
      printf("%c", printableCost(costmap.getCost(j,i) ));
    }
    printf("\n");
  }
}

unsigned int countValues( costmap_2d::Costmap2D& costmap, unsigned char value, bool equal=true)
{
  unsigned int count = 0;
  for(int i=0;i<costmap.getSizeInCellsY();i++){
    for(int j=0;j<costmap.getSizeInCellsX();j++){
      unsigned char c = costmap.getCost(j,i);
      if((equal && c==value) || (!equal && c!=value))
      {
        count+=1;
      }
    }
  }
  return count;
}
#endif
