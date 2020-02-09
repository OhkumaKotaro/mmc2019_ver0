#include "Map.h"
#include "MazeCon.h"
#include <stdint.h>

void Map_AddUnknownWall(unsigned char x, unsigned char y, wallData_t *wall,unsigned char goal_size,unsigned char gx,unsigned char gy) {
	//When the column does not have more than three boards
	//no.1 colum
	if (x > 0 && y > 0) {
		if ((((wall->vertical[x] >> y)&0b1) | ((wall->horizontal[y] >> x)&0b1) | ((wall->vertical[x] >> (y - 1))&0b1) | ((wall->horizontal[y] >> (x - 1))&0b1))==0) {
			if ((wall->vertical_known[x] >> (y - 1))&0b1) {
				wall->horizontal[y] |= (1 << (x - 1));
				wall->horizontal_known[y] |= (1 << (x - 1));
			}
			else if ((wall->horizontal_known[y] >> (x - 1))&0b1) {
				wall->vertical[x] |= (1 << (y - 1));
				wall->vertical_known[x] |= (1 << (y - 1));
			}
		}
	}
	//no.2 colum
	if (x > 0 && y < (MAZE_SIZE - 1)) {
		if ((((wall->vertical[x] >> y)&0b1) | ((wall->horizontal[y+1] >> (x-1))&0b1) | ((wall->vertical[x] >> (y + 1))&0b1) | ((wall->horizontal[y+1] >> x)&0b1))==0) {
			if ((wall->vertical_known[x] >> (y + 1))&0b1) {
				wall->horizontal[y+1] |= (1 << (x - 1));
				wall->horizontal_known[y + 1] |= (1 << (x - 1));
			}
			else if ((wall->horizontal_known[y+1] >> (x - 1))&0b1) {
				wall->vertical[x] |= (1 << (y + 1));
				wall->vertical_known[x] |= (1 << (y + 1));
			}
		}
	}
	//no.3 colum
	if (x < (MAZE_SIZE-1) && y < (MAZE_SIZE - 1)) {
		if ((((wall->vertical[x+1] >> y)&0b1) | ((wall->horizontal[y + 1] >> x)&0b1) | ((wall->vertical[x+1] >> (y + 1))&0b1) | ((wall->horizontal[y + 1] >> (x+1))&0b1))==0) {
			if ((wall->vertical_known[x+1] >> (y + 1))&0b1) {
				wall->horizontal[y + 1] |= (1 << (x + 1));
				wall->horizontal_known[y + 1] |= (1 << (x + 1));
			}
			else if ((wall->horizontal_known[y + 1] >> (x + 1))&0b1) {
				wall->vertical[x+1] |= (1 << (y + 1));
				wall->vertical_known[x+1] |= (1 << (y + 1));
			}
		}
	}
	//no.4 colum
	if (x < (MAZE_SIZE - 1) && y > 0) {
		if ((((wall->vertical[x + 1] >> y)&0b1) | ((wall->horizontal[y] >> x)&0b1) | ((wall->vertical[x + 1] >> (y - 1))&0b1) | ((wall->horizontal[y] >> (x + 1))&0b1))==0) {
			if ((wall->vertical_known[x+1] >> (y - 1))&0b1) {
				wall->horizontal[y] |= (1 << (x + 1));
				wall->horizontal_known[y] |= (1 << (x + 1));
			}
			else if ((wall->horizontal_known[y] >> (x + 1))&0b1) {
				wall->vertical[x+1] |= (1 << (y - 1));
				wall->vertical_known[x + 1] |= (1 << (y - 1));
			}
		}
	}

	if (goal_size == 4) {
		wall->horizontal[gy + 1] &= (~(1 << gx));
		wall->vertical[gx + 1] &= (~(1 << gy));
		wall->horizontal[gy + 1] &= (~(1 << (gx + 1)));
		wall->vertical[gx + 1] &= (~(1 << (gy + 1)));
	}


	//remove blind alley
	if (goal_size == 4) {
		unsigned char que_x[MAX_STEP];
		unsigned char que_y[MAX_STEP];
		unsigned char head = 0;
		unsigned char tail = 0;

		que_x[head] = x;
		que_y[head] = y;
		head++;
		while (head != tail) {
			//deque
			unsigned char buff_x = que_x[tail];
			unsigned char buff_y = que_y[tail];
			tail++;
			//no.1 blind alley
			if (buff_x > 0) {
				if (((wall->vertical[buff_x] >> buff_y) & 0b1)) {
					if ((((wall->vertical[buff_x - 1] >> buff_y) & 0b1) + ((wall->horizontal[buff_y + 1] >> (buff_x - 1)) & 0b1) + ((wall->horizontal[buff_y] >> (buff_x - 1)) & 0b1)) == 2) {
						if (!(((wall->vertical_known[buff_x - 1] >> buff_y) & 0b1) & ((wall->horizontal_known[buff_y + 1] >> (buff_x - 1)) & 0b1) & ((wall->horizontal_known[buff_y] >> (buff_x - 1)) & 0b1))) {
							wall->vertical[buff_x - 1] |= (1 << buff_y);
							//wall->vertical_known[buff_x - 1] |= (1 << buff_y);

							wall->horizontal[buff_y + 1] |= (1 << (buff_x - 1));
							//wall->horizontal_known[buff_y + 1] |= (1 << (buff_x - 1));

							wall->horizontal[buff_y] |= (1 << (buff_x - 1));
							//wall->horizontal_known[buff_y] |= (1 << (buff_x - 1));

							//enque
							que_x[head] = buff_x - 1;
							que_y[head] = buff_y;
							head++;
						}
					}
				}
			}
			//no.2 blind alley
			if (buff_y < (MAZE_SIZE - 1)) {
				if (((wall->horizontal[buff_y + 1] >> buff_x) & 0b1)) {
					if ((((wall->vertical[buff_x] >> (buff_y + 1)) & 0b1) + ((wall->horizontal[buff_y + 2] >> buff_x) & 0b1) + ((wall->vertical[buff_x + 1] >> (buff_y + 1)) & 0b1)) == 2) {
						if (!(((wall->vertical_known[buff_x] >> (buff_y + 1)) & 0b1) & ((wall->horizontal_known[buff_y + 2] >> buff_x) & 0b1) & ((wall->vertical_known[buff_x + 1] >> (buff_y + 1)) & 0b1))) {
							wall->vertical[buff_x] |= (1 << (buff_y + 1));
							//wall->vertical_known[buff_x] |= (1 << (buff_y + 1));

							wall->horizontal[buff_y + 2] |= (1 << buff_x);
							//wall->horizontal_known[buff_y + 2] |= (1 << buff_x);

							wall->vertical[buff_x + 1] |= (1 << (buff_y + 1));
							//wall->vertical_known[buff_x + 1] |= (1 << (buff_y + 1));

							//enque
							que_x[head] = buff_x;
							que_y[head] = buff_y + 1;
							head++;
						}
					}
				}
			}
			//no.3 blind alley
			if (buff_x < (MAZE_SIZE - 1)) {
				if (((wall->vertical[buff_x + 1] >> buff_y) & 0b1)) {
					if ((((wall->vertical[buff_x + 2] >> buff_y) & 0b1) + ((wall->horizontal[buff_y + 1] >> (buff_x + 1)) & 0b1) + ((wall->horizontal[buff_y] >> (buff_x + 1)) & 0b1)) == 2) {
						if (!(((wall->vertical_known[buff_x + 2] >> buff_y) & 0b1) & ((wall->horizontal_known[buff_y + 1] >> (buff_x + 1)) & 0b1) & ((wall->horizontal_known[buff_y] >> (buff_x + 1)) & 0b1))) {
							wall->vertical[buff_x + 2] |= (1 << buff_y);
							//wall->vertical_known[buff_x + 2] |= (1 << buff_y);

							wall->horizontal[buff_y + 1] |= (1 << (buff_x + 1));
							//wall->horizontal_known[buff_y + 1] |= (1 << (buff_x + 1));

							wall->horizontal[buff_y] |= (1 << (buff_x + 1));
							//wall->horizontal_known[buff_y] |= (1 << (buff_x + 1));

							//enque
							que_x[head] = buff_x + 1;
							que_y[head] = buff_y;
							head++;
						}
					}
				}
			}
			//no.4 blind alley
			if (buff_y > 0) {
				if (((wall->horizontal[buff_y] >> buff_x) & 0b1)) {
					if ((((wall->vertical[buff_x] >> (buff_y - 1)) & 0b1) + ((wall->horizontal[buff_y - 1] >> buff_x) & 0b1) + ((wall->vertical[buff_x + 1] >> (buff_y - 1)) & 0b1)) == 2) {
						if (!(((wall->vertical_known[buff_x] >> (buff_y - 1)) & 0b1) & ((wall->horizontal_known[buff_y - 1] >> buff_x) & 0b1) & ((wall->vertical_known[buff_x + 1] >> (buff_y - 1)) & 0b1))) {
							wall->vertical[buff_x] |= (1 << (buff_y - 1));
							//wall->vertical_known[buff_x] |= (1 << (buff_y - 1));

							wall->horizontal[buff_y - 1] |= (1 << buff_x);
							//wall->horizontal_known[buff_y - 1] |= (1 << buff_x);

							wall->vertical[buff_x + 1] |= (1 << (buff_y - 1));
							//wall->vertical_known[buff_x + 1] |= (1 << (buff_y - 1));

							//enque
							que_x[head] = buff_x;
							que_y[head] = buff_y - 1;
							head++;
						}
					}
				}
			}
		}
	}
}

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