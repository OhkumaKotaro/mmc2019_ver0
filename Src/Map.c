#include "Map.h"
#include "MazeCon.h"
#include <stdint.h>

void Map_addWall(wallData_t *wall, pos_t *pos, unsigned char n_wall, unsigned char e_wall, unsigned char w_wall, unsigned char s_wall)
{
	unsigned char x = pos->x;
	unsigned char y = pos->y;

	if (n_wall)
	{
		//Add north wall
		wall->horizontal[y + 1] |= (1<<x);
	}
	wall->horizontal_known[y + 1] |= (1<<x);

	if (e_wall)
	{
		//Add east wall
		wall->vertical[x + 1] |= (1<<y);
	}
	wall->vertical_known[x + 1] |= (1<<y);

	if (s_wall)
	{
		//Add south wall
		wall->horizontal[y] |= (1<<x);
	}
	wall->horizontal_known[y] |= (1<<x);

	if (w_wall)
	{
		//Add west wall
		wall->vertical[x] |= (1<<y);
	}
	wall->vertical_known[x] |= (1<<y);
}

void Map_DelWall(wallData_t *wall, pos_t *pos, unsigned char n_wall, unsigned char e_wall, unsigned char w_wall, unsigned char s_wall)
{
	unsigned char x = pos->x;
	unsigned char y = pos->y;

	if (n_wall == 0)
	{
		//Remobe north wall
		wall->horizontal[y + 1] &= (~(1<<x));
	}
	wall->horizontal_known[y + 1] |= (1<<x);

	if (e_wall == 0)
	{
		//Remove east wall
		wall->vertical[x + 1] &= (~(1<<y));
	}
	wall->vertical_known[x + 1] |= (1<<y);

	if (s_wall == 0)
	{
		//Remobe south wall
		wall->horizontal[y] &= (~(1<<x));
	}
	wall->horizontal_known[y] |= (1<<x);

	if (w_wall == 0)
	{
		//Remobe weat wall
		wall->vertical[x] &= (~(1<<y));
	}
	wall->vertical_known[x] |= 1<<y;
}

void Map_Init(wallData_t *wall)
{
	for (unsigned char i = 0; i <= MAZE_SIZE; i++)
	{
		wall->horizontal[i] = 0;
		wall->vertical[i] = 0;
		wall->horizontal_known[i] = 0;
		wall->vertical_known[i] = 0;
	}
	//�O�ǂ��Z�b�g
	wall->horizontal[0] = 0xffff;
	wall->vertical[0] = 0xffff;
	wall->horizontal[MAZE_SIZE] = 0xffff;
	wall->vertical[MAZE_SIZE] = 0xffff;
	//start���̕ǂ��Z�b�g����
	wall->vertical[1] = 0x0001;
	wall->vertical_known[1] = 0x0001;
	//�O�ǂ�T���ς݂ɂ���
	wall->horizontal_known[0] = 0xffff;
	wall->vertical_known[0] = 0xffff;
	wall->horizontal_known[MAZE_SIZE] = 0xffff;
	wall->vertical_known[MAZE_SIZE] = 0xffff;
}

void Map_InitFast(wallData_t *wall)
{
	for (unsigned char i = 0; i <= MAZE_SIZE; i++)
	{
		wall->horizontal[i] = 0xffff;
		wall->vertical[i] = 0xffff;
		wall->horizontal_known[i] = 0;
		wall->vertical_known[i] = 0;
	}
	//�O�ǂ��Z�b�g
	wall->horizontal[0] = 0xffff;
	wall->vertical[0] = 0xffff;
	wall->horizontal[MAZE_SIZE] = 0xffff;
	wall->vertical[MAZE_SIZE] = 0xffff;
	//start���̕ǂ��Z�b�g����
	wall->vertical_known[1] = 0x0001;
	//�O�ǂ�T���ς݂ɂ���
	wall->horizontal_known[0] = 0xffff;
	wall->vertical_known[0] = 0xffff;
	wall->horizontal_known[MAZE_SIZE] = 0xffff;
	wall->vertical_known[MAZE_SIZE] = 0xffff;
}