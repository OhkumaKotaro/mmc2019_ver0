#ifndef __MAZE_H
#define __MAZE_H
#ifdef __cplusplus
extern "C"
{
#endif
#include"MazeCon.h"
#include <stdint.h>

#define FALSE 0
#define TRUE 1
	void Maze_UpdatePosition(unsigned char *dir, pos_t *pos);
	unsigned char Maze_GetWallData(unsigned char x, unsigned char y, unsigned char dir, wallData_t *wall);
	void Maze_UpdateStepMap(unsigned char *goal_flag, unsigned char gx, unsigned char gy, wallData_t *wall);
	unsigned char Maze_GetNextMotion(pos_t *mypos, wallData_t *wall);
	unsigned char Maze_GetStep(unsigned char x, unsigned char y);
	void Plan_Root(unsigned char motion[MAX_STEP], wallData_t *wall, unsigned char *tail);
	void Plan_Compress(unsigned char *a_qmotion, unsigned char *head, unsigned char *tail);
	void Maze_Printf(void);
#ifdef __cplusplus
}
#endif
#endif