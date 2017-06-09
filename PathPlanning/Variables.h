#ifndef _VARIABLES_TO_DEFINE
#define _VARIABLES_TO_DEFINE

#define PI 3.141592653589793
#define eps1 2.2204e-016
#define infcon 10000000
#define Grid 0.2
#define MAP_WIDTH 150
#define MAP_HEIGHT 400
#define Grid_NUM_X MAP_WIDTH / Grid
#define Grid_NUM_Y MAP_WIDTH / Grid
#define X_START -MAP_WIDTH / 2
#define Y_START MAP_HEIGHT / 2
#define CAR_WIDTH 11
#define CAR_HEIGHT 15


//for grid clothoid search map
#define GRID_START -30
#define GRID_END 30
#define ANGLE_START 70
#define ANGLE_END 110

//define the direction
#define TURN_LEFT -1
#define TUN_RIGHT 1

#define MAX_INT 1<<16-1;

#define SAFE_REGION_START 4
#define SAFE_REGION_END 6

#endif