#ifndef __MAP_H
#define __MAP_H
#ifdef __cplusplus
extern "C"
{
#endif
	#include "MazeCon.h"

	void Map_AddUnknownWall(unsigned char x, unsigned char y, wallData_t *wall, unsigned char goal_size, unsigned char gx, unsigned char gy);
	void Map_addWall(wallData_t *wall, pos_t *pos, unsigned char n_wall, unsigned char e_wall, unsigned char s_wall, unsigned char w_wall);
	void Map_DelWall(wallData_t *wall, pos_t *pos, unsigned char n_wall, unsigned char e_wall, unsigned char w_wall, unsigned char s_wall);
	void Map_Init(wallData_t *wall);
	void Map_InitFast(wallData_t *wall);
	
#ifdef __cplusplus
}
#endif
#endif /* __MAP_H */
