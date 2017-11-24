#ifndef INC_RANGE_ROUTER_H
#define INC_RANGE_ROUTER_H

#include "Route_2pinnets.h"
#include "../misc/geometry.h"

#define INTERVAL_NUM 10
#define EXPAND_RANGE_SIZE 10
#define EXPAND_RANGE_INC 1

class Range_element
{
	public:
		int x1;
        int y1;
        int x2;
        int y2;
		
    public:
		Range_element(int x1, int y1, int x2, int y2)
            :x1(x1), y1(y1), x2(x2), y2(y2) {}
};

class Grid_edge_element
{
	public:
        Jm::Coordinate_2d* grid;
		int dir;
		
    public:
		Grid_edge_element(Jm::Coordinate_2d* grid, int dir)
            :grid(grid), dir(dir) {}
};

class Interval_element
{
	public:
		double begin_value;
		double end_value;
		vector<Grid_edge_element*> grid_edge_vector;
};

//EXTERN FUNCTIONS
extern void define_interval();
extern void divide_grid_edge_into_interval();
extern void specify_all_range(void);

#endif //INC_RANGE_ROUTER_H
