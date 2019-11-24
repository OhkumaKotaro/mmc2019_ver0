
#include "Maze.h"
#include "stdint.h"
#include "MazeCon.h"
//dx library
#include "stdio.h"

#define MAX_STEP_EX 0xffff

unsigned char step[MAZE_SIZE][MAZE_SIZE];
volatile uint16_t stepEx_h[MAZE_SIZE + 1][MAZE_SIZE + 1];
volatile uint16_t stepEx_v[MAZE_SIZE + 1][MAZE_SIZE + 1];
unsigned char goal_size = 4;

uint8_t Maze_GetGoalSize(void) {
	return goal_size;
}
void Maze_UpdatePosition(uint16_t dir, pos_t *pos)
{
	if (dir > 3)
	{
		dir = 3;
	}
	// ���̓���Ō����A���W���X�V����
	if (dir == FRONT)
	{
		switch (pos->dir)
		{
		case NORTH:
			pos->y++;
			break;
		case EAST:
			pos->x++;
			break;
		case SOUTH:
			pos->y--;
			break;
		case WEST:
			pos->x--;
			break;
		default:
			break;
		}
	}
	else if (dir == LEFT)
	{
		switch (pos->dir)
		{
		case NORTH:
			pos->x--;
			pos->dir = WEST;
			break;
		case EAST:
			pos->y++;
			pos->dir = NORTH;
			break;
		case SOUTH:
			pos->x++;
			pos->dir = EAST;
			break;
		case WEST:
			pos->y--;
			pos->dir = SOUTH;
			break;
		default:
			break;
		}
	}
	else if (dir == RIGHT)
	{
		switch (pos->dir)
		{
		case NORTH:
			pos->x++;
			pos->dir = EAST;
			break;
		case EAST:
			pos->y--;
			pos->dir = SOUTH;
			break;
		case SOUTH:
			pos->x--;
			pos->dir = WEST;
			break;
		case WEST:
			pos->y++;
			pos->dir = NORTH;
			break;
		default:
			break;
		}
	}
	else if (dir == REAR)
	{
		switch (pos->dir)
		{
		case NORTH:
			pos->y--;
			pos->dir = SOUTH;
			break;
		case EAST:
			pos->x--;
			pos->dir = WEST;
			break;
		case SOUTH:
			pos->y++;
			pos->dir = NORTH;
			break;
		case WEST:
			pos->x++;
			pos->dir = EAST;
			break;
		default:
			break;
		}
	}
}

unsigned char Maze_GetWallData(unsigned char x, unsigned char y, unsigned char dir, wallData_t *wall)
{
	uint16_t check_wall = 1;

	if (dir > 3)
	{
		dir = dir - 4;
	}

	if (dir == NORTH)
	{
		check_wall <<= x;
		check_wall &= wall->horizontal[y + 1];
		if (check_wall != 0)
		{
			check_wall = 1;
		}
	}
	else if (dir == EAST)
	{
		check_wall <<= y;
		check_wall &= wall->vertical[x + 1];
		if (check_wall != 0)
		{
			check_wall = 1;
		}
	}
	else if (dir == SOUTH)
	{
		check_wall <<= x;
		check_wall &= wall->horizontal[y];
		if (check_wall != 0)
		{
			check_wall = 1;
		}
	}
	else if (dir == WEST)
	{
		check_wall <<= y;
		check_wall &= wall->vertical[x];
		if (check_wall != 0)
		{
			check_wall = 1;
		}
	}

	if (check_wall == 1)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void Search_UnknownWall(wallData_t *wall, unsigned char *tail, unsigned char pos[MAX_STEP])
{
	unsigned char buff_tail = *tail;
	//init step
	for (unsigned char i = 0; i < MAZE_SIZE; i++)
	{
		for (unsigned char j = 0; j < MAZE_SIZE; j++)
		{
			unsigned char tmp_step = 255;
			//north
			if ((wall->horizontal_known[j + 1] & (0b1 << i)) == 0)
			{
				tmp_step = 0;
			}
			//south
			if (j > 0)
			{
				if ((wall->horizontal_known[j] & (0b1 << i)) == 0)
				{
					tmp_step = 0;
				}
			}
			//east
			if ((wall->vertical_known[i + 1] & (0b1 << j)) == 0)
			{
				tmp_step = 0;
			}
			//west
			if (i > 0)
			{
				if ((wall->vertical_known[i] & (0b1 << j)) == 0)
				{
					tmp_step = 0;
				}
			}
			if (tmp_step == 0)
			{
				*(pos + buff_tail) = (i << 4) | j;
				buff_tail++;
			}
			step[i][j] = tmp_step;
		}
	}
	*tail = buff_tail;
}

void Maze_UpdateStepMap(unsigned char *goal_flag, unsigned char gx, unsigned char gy, wallData_t *wall)
{
	unsigned char head = 0;
	unsigned char tail = 0;
	unsigned char pos[MAX_STEP];

	//unsigned char step[MAZE_SIZE][MAZE_SIZE];

	for (unsigned char x = 0; x < MAZE_SIZE; x++)
	{
		for (unsigned char y = 0; y < MAZE_SIZE; y++)
		{
			step[x][y] = MAX_STEP;
		}
	}

	if (*goal_flag == 1)
	{
		Search_UnknownWall(wall, &tail, pos);
	}
	else
	{
		if (goal_size == 4)
		{
			step[gx + 1][gy] = 0;
			step[gx + 1][gy + 1] = 0;
			step[gx][gy + 1] = 0;
			step[gx][gy] = 0;

			pos[tail] = gx << 4 | gy;
			tail++;
			pos[tail] = (gx + 1) << 4 | gy;
			tail++;
			pos[tail] = (gx + 1) << 4 | (gy + 1);
			tail++;
			pos[tail] = (gx + 1) << 4 | (gy + 1);
			tail++;
		}
		else
		{
			step[gx][gy] = 0;
			pos[tail] = (gx << 4) | gy;
			tail++;
		}
	}

	while (head < tail)
	{
		unsigned int tmp;
		unsigned char y = pos[head] & 0b00001111;
		unsigned char x = (pos[head] & 0b11110000) >> 4;
		head++;
		//north
		if (y < MAZE_SIZE - 1)
		{
			tmp = (wall->horizontal[y + 1] & (0b1 << x));
			if (tmp == 0 && (step[x][y + 1] == MAX_STEP))
			{
				step[x][y + 1] = step[x][y] + 1;
				pos[tail] = (x << 4) | (y + 1);
				tail++;
			}
		}
		//east
		if (x < MAZE_SIZE - 1)
		{
			tmp = (wall->vertical[x + 1] & (0b1 << y));
			if (tmp == 0 && (step[x + 1][y] == MAX_STEP))
			{
				step[x + 1][y] = step[x][y] + 1;
				pos[tail] = ((x + 1) << 4) | y;
				tail++;
			}
		}

		//south
		if (y > 0)
		{
			tmp = (wall->horizontal[y] & (0b1 << x));
			if (tmp == 0 && (step[x][y - 1] == MAX_STEP))
			{
				step[x][y - 1] = step[x][y] + 1;
				pos[tail] = ((x << 4) | (y - 1));
				tail++;
			}
		}

		//west
		if (x > 0)
		{
			tmp = (wall->vertical[x] & (0b1 << y));
			if (tmp == 0 && (step[x - 1][y] == MAX_STEP))
			{
				step[x - 1][y] = step[x][y] + 1;
				pos[tail] = (((x - 1) << 4) | y);
				tail++;
			}
		}
	}
	if (step[0][0] == MAX_STEP)
	{
		*goal_flag = 2;
	}
}

unsigned char Maze_GetNextMotion(pos_t *mypos, wallData_t *wall)
{
	unsigned char tmp_step = MAX_STEP; // ����
	unsigned char tmp_dir = REAR;	  // ����
									   // ���݂̌����ɉ����ďꍇ�������A ���������Ȃ������𔻒f
									   // ���H�O�ɐi�ނ̂ƃS�[�����X�^�[�g�}�X�ȊO�̏ꍇ(0,0)�ɐi�ނ̂�j�~

	unsigned char x = mypos->x;
	unsigned char y = mypos->y;

	switch (mypos->dir)
	{
	case NORTH:
		if (step[x][y + 1] < tmp_step)
		{
			if (y < MAZE_SIZE - 1)
			{
				if (((wall->horizontal[y + 1] >> x) & 0b1) == FALSE)
				{
					tmp_step = step[x][y + 1];
					tmp_dir = FRONT;
				}
			}
		}
		if (step[x - 1][y] < tmp_step)
		{
			if (x > 0)
			{
				if (((wall->vertical[x] >> y) & 0b1) == FALSE)
				{
					tmp_step = step[x - 1][y];
					tmp_dir = LEFT;
				}
			}
		}
		if (step[x + 1][y] < tmp_step)
		{
			if (x < MAZE_SIZE - 1)
			{
				if (((wall->vertical[x + 1] >> y) & 0b1) == FALSE)
				{
					tmp_step = step[x + 1][y];
					tmp_dir = RIGHT;
				}
			}
		}
		if (tmp_step == MAX_STEP || step[x][y - 1] < tmp_step)
		{
			if (((wall->horizontal[y + 1] >> x) & 0b1) == FALSE)
			{
				tmp_dir = REAR;
			}
			else
			{
				tmp_dir = PIVO_REAR;
			}
		}
		break;
	case EAST:
		if (step[x + 1][y] < tmp_step)
		{
			if (x < MAZE_SIZE - 1)
			{
				if (((wall->vertical[x + 1] >> y) & 0b1) == FALSE)
				{
					tmp_step = step[x + 1][y];
					tmp_dir = FRONT;
				}
			}
		}
		if (step[x][y + 1] < tmp_step)
		{
			if (y < MAZE_SIZE - 1)
			{
				if (((wall->horizontal[y + 1] >> x) & 0b1) == FALSE)
				{
					tmp_step = step[x][y + 1];
					tmp_dir = LEFT;
				}
			}
		}
		if (step[x][y - 1] < tmp_step)
		{
			if (y > 0)
			{
				if (((wall->horizontal[y] >> x) & 0b1) == FALSE)
				{
					tmp_step = step[x][y - 1];
					tmp_dir = RIGHT;
				}
			}
		}
		if (tmp_step == MAX_STEP || step[x - 1][y] < tmp_step)
		{
			if (((wall->vertical[x + 1] >> y) & 0b1) == FALSE)
			{
				tmp_dir = REAR;
			}
			else
			{
				tmp_dir = PIVO_REAR;
			}
		}
		break;
	case SOUTH:
		if (step[x][y - 1] < tmp_step)
		{
			if (y > 0)
			{
				if (((wall->horizontal[y] >> x) & 0b1) == FALSE)
				{
					tmp_step = step[x][y - 1];
					tmp_dir = FRONT;
				}
			}
		}
		if (step[x + 1][y] < tmp_step)
		{
			if (x < MAZE_SIZE - 1)
			{
				if (((wall->vertical[x + 1] >> y) & 0b1) == FALSE)
				{
					tmp_step = step[x + 1][y];
					tmp_dir = LEFT;
				}
			}
		}
		if (step[x - 1][y] < tmp_step)
		{
			if (x > 0)
			{
				if (((wall->vertical[x] >> y) & 0b1) == FALSE)
				{
					tmp_step = step[x - 1][y];
					tmp_dir = RIGHT;
				}
			}
		}
		if (tmp_step == MAX_STEP || step[x][y + 1] < tmp_step)
		{
			if (((wall->horizontal[y] >> x) & 0b1) == FALSE)
			{
				tmp_dir = REAR;
			}
			else
			{
				tmp_dir = PIVO_REAR;
			}
		}
		break;
	case WEST:
		if (step[x - 1][y] < tmp_step)
		{
			if (x > 0)
			{
				if (((wall->vertical[x] >> y) & 0b1) == FALSE)
				{
					tmp_step = step[x - 1][y];
					tmp_dir = FRONT;
				}
			}
		}
		if (step[x][y - 1] < tmp_step)
		{
			if (y > 0)
			{
				if (((wall->horizontal[y] >> x) & 0b1) == FALSE)
				{
					tmp_step = step[x][y - 1];
					tmp_dir = LEFT;
				}
			}
		}
		if (step[x][y + 1] < tmp_step)
		{
			if (y < MAZE_SIZE - 1)
			{
				if (((wall->horizontal[y + 1] >> x) & 0b1) == FALSE)
				{
					tmp_step = step[x][y + 1];
					tmp_dir = RIGHT;
				}
			}
		}
		if (tmp_step == MAX_STEP || step[x + 1][y] < tmp_step)
		{
			if (((wall->vertical[x] >> y) & 0b1) == FALSE)
			{
				tmp_dir = REAR;
			}
			else
			{
				tmp_dir = PIVO_REAR;
			}
		}
		break;
	default:
		break;
	}
	return tmp_dir;
}

unsigned char Maze_GetStep(unsigned char x, unsigned char y)
{
	return step[x][y];
}

void Maze_UpdateStepMapEx(wallData_t *wallDate, uint16_t weight_s, uint16_t weight_t, uint8_t gx, uint8_t gy)
{
	uint8_t head_h = 0;
	uint8_t head_v = 0;
	uint8_t tail_h = 0;
	uint8_t tail_v = 0;
	uint8_t x_h[0xff];
	uint8_t y_h[0xff];
	uint8_t x_v[0xff];
	uint8_t y_v[0xff];
	//init step
	wallDate->horizontal[0] = 0xffff;
	wallDate->horizontal[16] = 0xffff;
	wallDate->vertical[0] = 0xffff;
	wallDate->vertical[16] = 0xffff;
	for (uint8_t i = 0; i <= MAZE_SIZE; i++)
	{
		for (uint8_t j = 0; j <= MAZE_SIZE; j++)
		{
			stepEx_h[i][j] = MAX_STEP_EX;
			stepEx_v[i][j] = MAX_STEP_EX;
		}
	}
	//add goal step
	if (goal_size == 1)
	{
		//horizontal step
		if ((wallDate->horizontal[gy] & (0b1 << gx)) == FALSE)
		{
			stepEx_h[gx][gy] = 0;
			x_h[tail_h] = gx;
			y_h[tail_h] = gy;
			tail_h++;
		}
		if ((wallDate->horizontal[gy + 1] & (0b1 << gx)) == FALSE)
		{
			stepEx_h[gx][gy + 1] = 0;
			x_h[tail_h] = gx;
			y_h[tail_h] = gy + 1;
			tail_h++;
		}
		//vertical step
		if ((wallDate->vertical[gx] & (0b1 << gy)) == FALSE)
		{
			stepEx_v[gx][gy] = 0;
			x_v[tail_v] = gx;
			y_v[tail_v] = gy;
			tail_v++;
		}
		if ((wallDate->vertical[gx + 1] & (0b1 << gy)) == FALSE)
		{
			stepEx_v[gx + 1][gy] = 0;
			x_v[tail_v] = gx + 1;
			y_v[tail_v] = gy;
			tail_v++;
		}
	}
	else if (goal_size == 4)
	{
		//horizontal step
		if ((wallDate->horizontal[gy] & (0b1 << gx)) == FALSE)
		{
			stepEx_h[gx][gy] = 0;
			x_h[tail_h] = gx;
			y_h[tail_h] = gy;
			tail_h++;
		}
		if ((wallDate->horizontal[gy] & (0b1 << (gx + 1))) == FALSE)
		{
			stepEx_h[gx + 1][gy] = 0;
			x_h[tail_h] = gx + 1;
			y_h[tail_h] = gy;
			tail_h++;
		}

		if ((wallDate->horizontal[gy + 1] & (0b1 << gx)) == FALSE)
		{
			stepEx_h[gx][gy + 1] = 0;
			x_h[tail_h] = gx;
			y_h[tail_h] = gy + 1;
			tail_h++;
		}
		if ((wallDate->horizontal[gy + 1] & (0b1 << (gx + 1))) == FALSE)
		{
			stepEx_h[gx + 1][gy + 1] = 0;
			x_h[tail_h] = gx + 1;
			y_h[tail_h] = gy + 1;
			tail_h++;
		}

		if ((wallDate->horizontal[gy + 2] & (0b1 << gx)) == FALSE)
		{
			stepEx_h[gx][gy + 2] = 0;
			x_h[tail_h] = gx;
			y_h[tail_h] = gy + 2;
			tail_h++;
		}
		if ((wallDate->horizontal[gy + 2] & (0b1 << (gx + 1))) == FALSE)
		{
			stepEx_h[gx + 1][gy + 2] = 0;
			x_h[tail_h] = gx + 1;
			y_h[tail_h] = gy + 2;
			tail_h++;
		}
		//vertical step
		if ((wallDate->vertical[gx] & (0b1 << gy)) == FALSE)
		{
			stepEx_v[gx][gy] = 0;
			x_v[tail_v] = gx;
			y_v[tail_v] = gy;
			tail_v++;
		}
		if ((wallDate->vertical[gx] & (0b1 << (gy + 1))) == FALSE)
		{
			stepEx_v[gx][gy + 1] = 0;
			x_v[tail_v] = gx;
			y_v[tail_v] = gy + 1;
			tail_v++;
		}

		if ((wallDate->vertical[gx + 1] & (0b1 << gy)) == FALSE)
		{
			stepEx_v[gx + 1][gy] = 0;
			x_v[tail_v] = gx + 1;
			y_v[tail_v] = gy;
			tail_v++;
		}
		if ((wallDate->vertical[gx + 1] & (0b1 << (gy + 1))) == FALSE)
		{
			stepEx_v[gx + 1][gy + 1] = 0;
			x_v[tail_v] = gx + 1;
			y_v[tail_v] = gy + 1;
			tail_v++;
		}

		if ((wallDate->vertical[gx + 2] & (0b1 << gy)) == FALSE)
		{
			stepEx_v[gx + 2][gy] = 0;
			x_v[tail_v] = gx + 2;
			y_v[tail_v] = gy;
			tail_v++;
		}
		if ((wallDate->vertical[gx + 2] & (0b1 << (gy + 1))) == FALSE)
		{
			stepEx_v[gx + 2][gy + 1] = 0;
			x_v[tail_v] = gx + 2;
			y_v[tail_v] = gy + 1;
			tail_v++;
		}
	}
	//update stepEx
	while (head_h != tail_h || head_v != tail_v)
	{
		//holizontal
		if (head_h != tail_h)
		{
			//holizontal->holizontal
			if (stepEx_h[x_h[head_h]][y_h[head_h]] != MAX_STEP_EX)
			{
				if (y_h[head_h] < (MAZE_SIZE - 1))
				{
					if ((wallDate->horizontal[y_h[head_h] + 1] & (0b1 << x_h[head_h])) == FALSE)
					{
						if (stepEx_h[x_h[head_h]][y_h[head_h] + 1] /*== MAX_STEP_EX*/ > stepEx_h[x_h[head_h]][y_h[head_h]] + weight_s)
						{
							stepEx_h[x_h[head_h]][y_h[head_h] + 1] = stepEx_h[x_h[head_h]][y_h[head_h]] + weight_s;
							uint8_t x_buff = x_h[head_h];
							uint8_t y_buff = y_h[head_h] + 1;
							x_h[tail_h] = x_buff;
							y_h[tail_h] = y_buff;
							tail_h++;
						}
					}
				}
				if (y_h[head_h] > 0)
				{
					if ((wallDate->horizontal[y_h[head_h] - 1] & (0b1 << x_h[head_h])) == FALSE)
					{
						if (stepEx_h[x_h[head_h]][y_h[head_h] - 1] /*== MAX_STEP_EX*/ > stepEx_h[x_h[head_h]][y_h[head_h]] + weight_s)
						{
							stepEx_h[x_h[head_h]][y_h[head_h] - 1] = stepEx_h[x_h[head_h]][y_h[head_h]] + weight_s;
							uint8_t x_buff = x_h[head_h];
							uint8_t y_buff = y_h[head_h] - 1;
							x_h[tail_h] = x_buff;
							y_h[tail_h] = y_buff;
							tail_h++;
						}
					}
				}
				//holizontal->vertical
				if (x_h[head_h] > 0)
				{
					if ((wallDate->vertical[x_h[head_h]] & (0b1 << y_h[head_h])) == FALSE)
					{
						if (stepEx_v[x_h[head_h]][y_h[head_h]] /*== MAX_STEP_EX*/ > stepEx_h[x_h[head_h]][y_h[head_h]] + weight_t)
						{
							stepEx_v[x_h[head_h]][y_h[head_h]] = stepEx_h[x_h[head_h]][y_h[head_h]] + weight_t;
							uint8_t x_buff = x_h[head_h];
							uint8_t y_buff = y_h[head_h];
							x_v[tail_v] = x_buff;
							y_v[tail_v] = y_buff;
							tail_v++;
						}
					}
				}
				if (y_h[head_h] > 0)
				{
					if ((wallDate->vertical[x_h[head_h]] & (0b1 << (y_h[head_h] - 1))) == FALSE)
					{
						if (stepEx_v[x_h[head_h]][y_h[head_h] - 1] /*== MAX_STEP_EX*/ > stepEx_h[x_h[head_h]][y_h[head_h]] + weight_t)
						{
							stepEx_v[x_h[head_h]][y_h[head_h] - 1] = stepEx_h[x_h[head_h]][y_h[head_h]] + weight_t;
							uint8_t x_buff = x_h[head_h];
							uint8_t y_buff = y_h[head_h] - 1;
							x_v[tail_v] = x_buff;
							y_v[tail_v] = y_buff;
							tail_v++;
						}
					}
				}
				if (x_h[head_h] < (MAZE_SIZE - 1))
				{
					if ((wallDate->vertical[x_h[head_h] + 1] & (0b1 << (y_h[head_h]))) == FALSE)
					{
						if (stepEx_v[x_h[head_h] + 1][y_h[head_h]] /*== MAX_STEP_EX*/ > stepEx_h[x_h[head_h]][y_h[head_h]] + weight_t)
						{
							stepEx_v[x_h[head_h] + 1][y_h[head_h]] = stepEx_h[x_h[head_h]][y_h[head_h]] + weight_t;
							uint8_t x_buff = x_h[head_h] + 1;
							uint8_t y_buff = y_h[head_h];
							x_v[tail_v] = x_buff;
							y_v[tail_v] = y_buff;
							tail_v++;
						}
					}
				}
				if ((y_h[head_h] > 0) && (x_h[head_h] < (MAZE_SIZE - 1)))
				{
					if ((wallDate->vertical[x_h[head_h] + 1] & (0b1 << (y_h[head_h] - 1))) == FALSE)
					{
						if (stepEx_v[x_h[head_h] + 1][y_h[head_h] - 1] == MAX_STEP_EX /*> stepEx_h[x_h[head_h]][y_h[head_h]] + weight_t*/)
						{
							stepEx_v[x_h[head_h] + 1][y_h[head_h] - 1] = stepEx_h[x_h[head_h]][y_h[head_h]] + weight_t;
							uint8_t x_buff = x_h[head_h] + 1;
							uint8_t y_buff = y_h[head_h] - 1;
							x_v[tail_v] = x_buff;
							y_v[tail_v] = y_buff;
							tail_v++;
						}
					}
				}
			}
			head_h++;
		}
		//vertical
		if (head_v != tail_v)
		{
			if (stepEx_v[x_v[head_v]][y_v[head_v]] != MAX_STEP_EX)
			{
				//vertical->vertical
				if (x_v[head_v] < MAZE_SIZE - 1)
				{
					if ((wallDate->vertical[x_v[head_v] + 1] & (0b1 << y_v[head_v])) == FALSE)
					{
						if (stepEx_v[x_v[head_v] + 1][y_v[head_v]] /*== MAX_STEP_EX*/ > stepEx_v[x_v[head_v]][y_v[head_v]] + weight_s)
						{
							stepEx_v[x_v[head_v] + 1][y_v[head_v]] = stepEx_v[x_v[head_v]][y_v[head_v]] + weight_s;
							uint8_t x_buff = x_v[head_v] + 1;
							uint8_t y_buff = y_v[head_v];
							x_v[tail_v] = x_buff;
							y_v[tail_v] = y_buff;
							tail_v++;
						}
					}
				}
				if (x_v[head_v] > 0)
				{
					if ((wallDate->vertical[x_v[head_v] - 1] & (0b1 << y_v[head_v])) == FALSE)
					{
						if (stepEx_v[x_v[head_v] - 1][y_v[head_v]] /*== MAX_STEP_EX */ > stepEx_v[x_v[head_v]][y_v[head_v]] + weight_s)
						{
							stepEx_v[x_v[head_v] - 1][y_v[head_v]] = stepEx_v[x_v[head_v]][y_v[head_v]] + weight_s;
							uint8_t x_buff = x_v[head_v] - 1;
							uint8_t y_buff = y_v[head_v];
							x_v[tail_v] = x_buff;
							y_v[tail_v] = y_buff;
							tail_v++;
						}
					}
				}
				//vertical->horizontal
				if (y_v[head_v] > 0)
				{
					if ((wallDate->horizontal[y_v[head_v]] & (0b1 << x_v[head_v])) == FALSE)
					{
						if (stepEx_h[x_v[head_v]][y_v[head_v]] /*== MAX_STEP_EX */ > stepEx_v[x_v[head_v]][y_v[head_v]] + weight_t)
						{
							stepEx_h[x_v[head_v]][y_v[head_v]] = stepEx_v[x_v[head_v]][y_v[head_v]] + weight_t;
							uint8_t x_buff = x_v[head_v];
							uint8_t y_buff = y_v[head_v];
							x_h[tail_h] = x_buff;
							y_h[tail_h] = y_buff;
							tail_h++;
						}
					}
				}
				if (x_v[head_v] > 0)
				{
					if ((wallDate->horizontal[y_v[head_v]] & (0b1 << (x_v[head_v] - 1))) == FALSE)
					{
						if (stepEx_h[x_v[head_v] - 1][y_v[head_v]] /*== MAX_STEP_EX */ > stepEx_v[x_v[head_v]][y_v[head_v]] + weight_t)
						{
							stepEx_h[x_v[head_v] - 1][y_v[head_v]] = stepEx_v[x_v[head_v]][y_v[head_v]] + weight_t;
							uint8_t x_buff = x_v[head_v] - 1;
							uint8_t y_buff = y_v[head_v];
							x_h[tail_h] = x_buff;
							y_h[tail_h] = y_buff;
							tail_h++;
						}
					}
				}
				if (y_v[head_v] < (MAZE_SIZE - 1))
				{
					if ((wallDate->horizontal[y_v[head_v] + 1] & (0b1 << x_v[head_v])) == FALSE)
					{
						if (stepEx_h[x_v[head_v]][y_v[head_v] + 1] /*== MAX_STEP_EX */ > stepEx_v[x_v[head_v]][y_v[head_v]] + weight_t)
						{
							stepEx_h[x_v[head_v]][y_v[head_v] + 1] = stepEx_v[x_v[head_v]][y_v[head_v]] + weight_t;
							uint8_t x_buff = x_v[head_v];
							uint8_t y_buff = y_v[head_v] + 1;
							x_h[tail_h] = x_buff;
							y_h[tail_h] = y_buff;
							tail_h++;
						}
					}
				}
				if (((y_v[head_v] < (MAZE_SIZE - 1)) && (x_v[head_v] > 0)))
				{
					if ((wallDate->horizontal[y_v[head_v] + 1] & (0b1 << (x_v[head_v] - 1))) == FALSE)
					{
						if (stepEx_h[x_v[head_v] - 1][y_v[head_v] + 1] /*== MAX_STEP_EX*/ > stepEx_v[x_v[head_v]][y_v[head_v]] + weight_t)
						{
							stepEx_h[x_v[head_v] - 1][y_v[head_v] + 1] = stepEx_v[x_v[head_v]][y_v[head_v]] + weight_t;
							uint8_t x_buff = x_v[head_v] - 1;
							uint8_t y_buff = y_v[head_v] + 1;
							x_h[tail_h] = x_buff;
							y_h[tail_h] = y_buff;
							tail_h++;
						}
					}
				}
			}
			head_v++;
		}
	}
}

uint16_t Maze_GetNextMotionEx(pos_t *mypos, wallData_t *wall)
{
	volatile uint16_t tmp_step = 0xffff; // ����
	uint16_t tmp_dir = 3;				 // ����
										 // ���݂̌����ɉ����ďꍇ�������A ���������Ȃ������𔻒f
										 // ���H�O�ɐi�ނ̂ƃS�[�����X�^�[�g�}�X�ȊO�̏ꍇ(0,0)�ɐi�ނ̂�j�~
	uint8_t x = mypos->x;
	uint8_t y = mypos->y;

	switch (mypos->dir)
	{
	case NORTH:
		if (stepEx_h[x][y + 1] < tmp_step)
		{
			if (y < MAZE_SIZE - 1)
			{
				if (((wall->horizontal[y + 1] >> x) & 0b1) == FALSE)
				{
					tmp_step = stepEx_h[x][y + 1];
					tmp_dir = 2 << 4 | FRONT;
				}
			}
		}
		if (stepEx_v[x][y] < tmp_step)
		{
			if (x > 0)
			{
				if (((wall->vertical[x] >> y) & 0b1) == FALSE)
				{
					tmp_step = stepEx_v[x][y];
					tmp_dir = LEFT;
				}
			}
		}
		if (stepEx_v[x + 1][y] < tmp_step)
		{
			if (x < MAZE_SIZE - 1)
			{
				if (((wall->vertical[x + 1] >> y) & 0b1) == FALSE)
				{
					tmp_step = stepEx_v[x + 1][y];
					tmp_dir = RIGHT;
				}
			}
		}
		break;
	case EAST:
		if (stepEx_v[x + 1][y] < tmp_step)
		{
			if (x < MAZE_SIZE - 1)
			{
				if (((wall->vertical[x + 1] >> y) & 0b1) == FALSE)
				{
					tmp_step = stepEx_v[x + 1][y];
					tmp_dir = 2 << 4 | FRONT;
				}
			}
		}
		if (stepEx_h[x][y + 1] < tmp_step)
		{
			if (y < MAZE_SIZE - 1)
			{
				if (((wall->horizontal[y + 1] >> x) & 0b1) == FALSE)
				{
					tmp_step = stepEx_h[x][y + 1];
					tmp_dir = LEFT;
				}
			}
		}
		if (stepEx_h[x][y] < tmp_step)
		{
			if (y > 0)
			{
				if (((wall->horizontal[y] >> x) & 0b1) == FALSE)
				{
					tmp_step = stepEx_h[x][y];
					tmp_dir = RIGHT;
				}
			}
		}
		break;
	case SOUTH:
		if (stepEx_h[x][y] < tmp_step)
		{
			if (y > 0)
			{
				if (((wall->horizontal[y] >> x) & 0b1) == FALSE)
				{
					tmp_step = stepEx_h[x][y];
					tmp_dir = 2 << 4 | FRONT;
				}
			}
		}
		if (stepEx_v[x + 1][y] < tmp_step)
		{
			if (x < MAZE_SIZE - 1)
			{
				if (((wall->vertical[x + 1] >> y) & 0b1) == FALSE)
				{
					tmp_step = stepEx_v[x + 1][y];
					tmp_dir = LEFT;
				}
			}
		}
		if (stepEx_v[x][y] < tmp_step)
		{
			if (x > 0)
			{
				if (((wall->vertical[x] >> y) & 0b1) == FALSE)
				{
					tmp_step = stepEx_v[x][y];
					tmp_dir = RIGHT;
				}
			}
		}
		break;
	case WEST:
		if (stepEx_v[x][y] < tmp_step)
		{
			if (x > 0)
			{
				if (((wall->vertical[x] >> y) & 0b1) == FALSE)
				{
					tmp_step = stepEx_v[x][y];
					tmp_dir = 2 << 4 | FRONT;
				}
			}
		}
		if (stepEx_h[x][y] < tmp_step)
		{
			if (y > 0)
			{
				if (((wall->horizontal[y] >> x) & 0b1) == FALSE)
				{
					tmp_step = stepEx_h[x][y];
					tmp_dir = LEFT;
				}
			}
		}
		if (stepEx_h[x][y + 1] < tmp_step)
		{
			if (y < MAZE_SIZE - 1)
			{
				if (((wall->horizontal[y + 1] >> x) & 0b1) == FALSE)
				{
					tmp_step = stepEx_h[x][y + 1];
					tmp_dir = RIGHT;
				}
			}
		}
		break;
	default:
		break;
	}
	return tmp_dir;
}

uint16_t Maze_KnownStepAccel(pos_t *mypos, wallData_t *wall, uint16_t next_motion)
{
	uint16_t motion = next_motion;
	uint16_t counter = 0;
	if (motion == FRONT)
	{
		while (1)
		{
			uint8_t x = mypos->x;
			uint8_t y = mypos->y;
			switch (mypos->dir)
			{
			case NORTH:
				y++;
				break;
			case EAST:
				x++;
				break;
			case SOUTH:
				y--;
				break;
			case WEST:
				x--;
				break;
			default:
				break;
			}

			uint16_t n_wall = ((wall->horizontal_known[y + 1] >> x) & 0b1);
			uint16_t e_wall = ((wall->vertical_known[x + 1] >> y) & 0b1);
			uint16_t s_wall = ((wall->horizontal_known[y] >> x) & 0b1);
			uint16_t w_wall = ((wall->vertical_known[x] >> y) & 0b1);

			if (1 == (n_wall & e_wall & s_wall & w_wall) && motion == FRONT)
			{
				counter++;
				Maze_UpdatePosition(FRONT, mypos);
				motion = Maze_GetNextMotion(mypos, wall);
			}
			else
			{
				if (counter == 0)
				{
					Maze_UpdatePosition(motion, mypos);
				}
				break;
			}
			if (motion != FRONT)
			{
				break;
			}
		}
		return counter << 4 | FRONT;
	}
	else
	{
		Maze_UpdatePosition(motion, mypos);
		return motion;
	}
}

uint16_t Maze_GetStepEx_h(uint8_t x, uint8_t y)
{
	return stepEx_h[x][y];
}

uint16_t Maze_GetStepEx_v(uint8_t x, uint8_t y)
{
	return stepEx_v[x][y];
}

void Compress_T90(uint16_t *motion, uint8_t *origin_tail)
{
	volatile uint8_t head = 0;
	volatile uint8_t tail = *origin_tail;
	volatile uint8_t buff_tail = 0;
	volatile uint16_t buff_motion[MAX_STEP];

	while (head < tail)
	{
		buff_motion[buff_tail] = *(motion + head);
		buff_tail++;
		if (*(motion + head) == LEFT || *(motion + head) == RIGHT)
		{
			head++;
			buff_motion[buff_tail] = *(motion + head);
			buff_tail++;
			if ((*(motion + head) == ((2 << 4) | FRONT)) && ((*(motion + head - 2) & 0xf) == FRONT))
			{
				buff_tail -= 3;
				if (buff_motion[buff_tail] == ((2 << 4) | FRONT))
				{
					buff_motion[buff_tail] = 1 << 4 | ADJUST;
					buff_tail++;
				}
				buff_motion[buff_tail] = T_90 << 4 | *(motion + head - 1);
				buff_tail++;
				buff_motion[buff_tail] = (1 << 4) | FRONT;
				buff_tail++;
			}
			else if (((*(motion + head) & 0xf) == ADJUST) && ((*(motion + head - 2) & 0xf) == FRONT))
			{
				buff_tail -= 3;
				if (buff_motion[buff_tail] == ((2 << 4) | FRONT))
				{
					buff_motion[buff_tail] = 1 << 4 | ADJUST;
					buff_tail++;
				}
				buff_motion[buff_tail] = T_90 << 4 | *(motion + head - 1);
				buff_tail++;
				head++;
				buff_motion[buff_tail] = *(motion + head);
				buff_tail++;
			}
			else if (*(motion + head) == GOAL && ((*(motion + head - 2) & 0xf) == FRONT))
			{
				buff_tail -= 3;
				if (buff_motion[buff_tail] == ((2 << 4) | FRONT))
				{
					buff_motion[buff_tail] = 1 << 4 | ADJUST;
					buff_tail++;
				}
				buff_motion[buff_tail] = (T_90 << 4) | *(motion + head - 1);
				buff_tail++;
				buff_motion[buff_tail] = GOAL;
				buff_tail++;
				break;
			}
		}
		head++;
	}
	for (uint8_t i = 0; i < buff_tail; i++)
	{
		motion[i] = buff_motion[i];
	}
	*origin_tail = buff_tail;
}

void Compress_T180(uint16_t *motion, uint8_t *origin_tail)
{
	uint8_t head = 0;
	uint8_t tail = *origin_tail;
	uint8_t buff_tail = 0;
	uint16_t buff_motion[MAX_STEP];

	while (head < tail)
	{
		buff_motion[buff_tail] = *(motion + head);
		buff_tail++;
		if (*(motion + head) == LEFT || *(motion + head) == RIGHT)
		{
			head++;
			buff_motion[buff_tail] = *(motion + head);
			buff_tail++;
			if (*(motion + head) == *(motion + head - 1))
			{
				head++;
				buff_motion[buff_tail] = *(motion + head);
				buff_tail++;
				if ((*(motion + head) == ((2 << 4) | FRONT)) && ((*(motion + head - 3) & 0xf) == FRONT))
				{
					buff_tail -= 4;
					if (buff_motion[buff_tail] == ((2 << 4) | FRONT))
					{
						buff_motion[buff_tail] = 1 << 4 | ADJUST;
						buff_tail++;
					}
					buff_motion[buff_tail] = (T_180 << 4) | *(motion + head - 1);
					buff_tail++;
					buff_motion[buff_tail] = 1 << 4 | FRONT;
					buff_tail++;
				}
				else if (((*(motion + head) & 0xf) == ADJUST) && ((*(motion + head - 3) & 0xf) == FRONT))
				{
					buff_tail -= 4;
					if (buff_motion[buff_tail] == ((2 << 4) | FRONT))
					{
						buff_motion[buff_tail] = 1 << 4 | ADJUST;
						buff_tail++;
					}
					buff_motion[buff_tail] = (T_180 << 4) | *(motion + head - 1);
					buff_tail++;
					head++;
					buff_motion[buff_tail] = *(motion + head);
					buff_tail++;
				}
				else if (*(motion + head) == GOAL)
				{
					buff_tail -= 4;
					if (buff_motion[buff_tail] == ((2 << 4) | FRONT) && ((*(motion + head - 3) & 0xf) == FRONT))
					{
						buff_motion[buff_tail] = 1 << 4 | ADJUST;
						buff_tail++;
					}
					buff_motion[buff_tail] = T_180 << 4 | *(motion + head - 1);
					buff_tail++;
					buff_motion[buff_tail] = GOAL;
					buff_tail++;
					break;
				}
			}
		}
		head++;
	}
	for (uint8_t i = 0; i < buff_tail; i++)
	{
		motion[i] = buff_motion[i];
	}
	*origin_tail = buff_tail;
}

void Compress_Diagonal(uint16_t *motion, uint8_t *origin_tail)
{
	volatile uint8_t head = 0;
	volatile uint8_t tail = *origin_tail;
	volatile uint8_t buff_tail = 0;
	volatile uint16_t buff_motion[MAX_STEP];

	while (head < tail)
	{
		buff_motion[buff_tail] = *(motion + head);
		buff_tail++;
		if (*(motion + head) == LEFT || *(motion + head) == RIGHT)
		{
			uint8_t step = 0;
			head++;
			buff_motion[buff_tail] = *(motion + head);
			buff_tail++;
			if (*(motion + head) == *(motion + head - 1))
			{
				buff_tail -= 3;
				if (buff_motion[buff_tail] == ((2 << 4) | FRONT))
				{
					buff_motion[buff_tail] = 1 << 4 | ADJUST;
					buff_tail++;
				}
				buff_motion[buff_tail] = T_135IN << 4 | *(motion + head);
				buff_tail++;
			}
			else if (*(motion + head) == LEFT || *(motion + head) == RIGHT)
			{
				buff_tail -= 3;
				if (buff_motion[buff_tail] == ((2 << 4) | FRONT))
				{
					buff_motion[buff_tail] = 1 << 4 | ADJUST;
					buff_tail++;
				}
				buff_motion[buff_tail] = T_45IN << 4 | *(motion + head - 1);
				buff_tail++;
				step = 1;
			}
			while (1)
			{
				head++;
				if (*(motion + head) == ((2 << 4) | FRONT))
				{
					if (step > 1)
					{
						buff_motion[buff_tail] = ((step - 1) << 4) | DIAGONAL;
						buff_tail++;
					}
					buff_motion[buff_tail] = T_45OUT << 4 | *(motion + head - 1);
					buff_tail++;
					buff_motion[buff_tail] = (1 << 4) | FRONT;
					buff_tail++;
					break;
				}
				else if ((*(motion + head) & 0xf) == ADJUST)
				{
					if (step > 1)
					{
						buff_motion[buff_tail] = ((step - 1) << 4) | DIAGONAL;
						buff_tail++;
					}
					buff_motion[buff_tail] = T_45OUT << 4 | *(motion + head - 1);
					buff_tail++;
					head++;
					buff_motion[buff_tail] = *(motion + head);
					buff_tail++;
					break;
				}
				else if (*(motion + head) == GOAL)
				{
					if (step > 1)
					{
						buff_motion[buff_tail] = ((step - 1) << 4) | DIAGONAL;
						buff_tail++;
					}
					buff_motion[buff_tail] = T_45OUT << 4 | *(motion + head - 1);
					buff_tail++;
					buff_motion[buff_tail] = GOAL;
					buff_tail++;
					break;
				}
				else if (*(motion + head) == *(motion + head - 1))
				{
					head++;
					if (*(motion + head) == ((2 << 4) | FRONT))
					{
						if (step > 1)
						{
							buff_motion[buff_tail] = ((step - 1) << 4) | DIAGONAL;
							buff_tail++;
						}
						buff_motion[buff_tail] = T_135OUT << 4 | *(motion + head - 1);
						buff_tail++;
						buff_motion[buff_tail] = (1 << 4) | FRONT;
						buff_tail++;
						break;
					}
					else if ((*(motion + head) & 0xf) == ADJUST)
					{
						if (step > 1)
						{
							buff_motion[buff_tail] = ((step - 1) << 4) | DIAGONAL;
							buff_tail++;
						}
						buff_motion[buff_tail] = T_135OUT << 4 | *(motion + head - 1);
						buff_tail++;
						head++;
						buff_motion[buff_tail] = *(motion + head);
						buff_tail++;
						break;
					}
					else if (*(motion + head) == GOAL)
					{
						if (step > 1)
						{
							buff_motion[buff_tail] = ((step - 1) << 4) | DIAGONAL;
							buff_tail++;
						}
						buff_motion[buff_tail] = T_135OUT << 4 | *(motion + head - 1);
						buff_tail++;
						buff_motion[buff_tail] = GOAL;
						buff_tail++;
						break;
					}
					else
					{
						if (step > 1)
						{
							buff_motion[buff_tail] = ((step - 1) << 4) | DIAGONAL;
							buff_tail++;
							step = 1;
						}
						buff_motion[buff_tail] = T_V90 << 4 | *(motion + head - 1);
						buff_tail++;
					}
				}
				else if (*(motion + head) == LEFT || *(motion + head) == RIGHT)
				{
					step++;
				}
			}
		}
		head++;
	}
	for (uint8_t i = 0; i < buff_tail; i++)
	{
		motion[i] = buff_motion[i];
	}
	*origin_tail = buff_tail;
}

void Maze_Compress(uint8_t mode_FastTurn, uint16_t *motion, uint32_t *velocity, uint8_t *origin_tail, float v_search, float v_fast)
{
	//add fast turn
	Compress_T90(motion, origin_tail);
	Compress_T180(motion, origin_tail);
	if (mode_FastTurn == TRUE)
	{
		Compress_Diagonal(motion, origin_tail);
	}
	volatile uint8_t head = 0;
	volatile uint8_t tail = *origin_tail;
	volatile uint8_t buff_tail = 0;
	volatile uint16_t buff_motion[255];
	volatile uint32_t buff_velocity;
	while (head < tail)
	{
		uint16_t step = 0;
		switch (*(motion + head) & 0xf)
		{
		case START:
			head++;
			while ((*(motion + head) & 0xf) == FRONT)
			{
				step += (*(motion + head) >> 4);
				head++;
			}
			buff_motion[buff_tail] = step << 4 | START;
			buff_tail++;
			if (*(motion + head) == LEFT || *(motion + head) == RIGHT)
			{
				if (step >= 2)
				{
					step -= 2;
					buff_tail--;
					buff_motion[buff_tail] = step << 4 | START;
					buff_tail++;
					buff_motion[buff_tail] = 2 << 4 | ADJUST;
					buff_tail++;
				}
			}
			break;
		case FRONT:
			while ((*(motion + head) & 0xf) == FRONT)
			{
				step += (*(motion + head) >> 4);
				head++;
			}
			if (*(motion + head) == GOAL)
			{
				buff_motion[buff_tail] = (step << 4) | GOAL;
				buff_tail++;
				head++;
			}
			else
			{
				buff_motion[buff_tail] = step << 4 | FRONT;
				buff_tail++;
				if (*(motion + head) == LEFT || *(motion + head) == RIGHT)
				{
					if (step > 2)
					{
						step -= 2;
						buff_tail--;
						buff_motion[buff_tail] = step << 4 | FRONT;
						buff_tail++;
						buff_motion[buff_tail] = 2 << 4 | ADJUST;
						buff_tail++;
					}
					else if (buff_motion[buff_tail - 2] == LEFT || buff_motion[buff_tail - 2] == RIGHT)
					{
						buff_tail--;
						buff_motion[buff_tail] = 2 << 4 | ADJUST;
						buff_tail++;
					}
				}
			}
			break;
		default:
			buff_motion[buff_tail] = *(motion + head);
			buff_tail++;
			head++;
			break;
		}
	}
	for (uint8_t i = 0; i < buff_tail; i++)
	{
		*(motion + i) = buff_motion[i];
	}

	tail = buff_tail;
	head = 0;
	buff_tail = 0;

	while (head < tail)
	{
		*(velocity + buff_tail) = (uint32_t)v_fast << 16 | (uint32_t)v_fast;
		if (*(motion + head - 1) == LEFT || *(motion + head - 1) == RIGHT)
		{
			buff_velocity = *(velocity + head) & 0xffff;
			*(velocity + buff_tail) = (uint32_t)v_search << 16 | buff_velocity;
		}
		if (*(motion + head + 1) == LEFT || *(motion + head + 1) == RIGHT)
		{
			if ((*(motion + head) & 0xf) == ADJUST)
			{
				*(velocity + buff_tail) = (uint32_t)v_search << 16 | (uint32_t)v_search;
				if ((*(motion + head - 1) & 0xf) == FRONT)
				{
					buff_velocity = *(velocity + head - 1) >> 16;
					*(velocity + buff_tail - 1) = buff_velocity << 16 | (uint32_t)v_search;
				}
			}
			else
			{
				buff_velocity = *(velocity + head) >> 16;
				*(velocity + buff_tail) = buff_velocity << 16 | (uint32_t)v_search;
			}
		}
		if (*(motion + head + 1) == GOAL && (*(motion + head) >> 4) != 0)
		{
			*(velocity + buff_tail) = (uint32_t)v_fast << 16 | 0;
			buff_tail++;
			break;
		}
		buff_tail++;
		head++;
	}
	*origin_tail = buff_tail;
	head = 0;
	while (head < buff_tail)
	{
		if ((*(motion + head) & 0xf) == DIAGONAL)
		{
			if ((*(motion + head + 1) & 0xf) == LEFT)
			{
				uint16_t buff_step = *(motion + head) >> 4;
				*(motion + head) = (buff_step << 4) | DIAGONAL_L;
			}
			else if ((*(motion + head + 1) & 0xf) == RIGHT)
			{
				uint16_t buff_step = *(motion + head) >> 4;
				*(motion + head) = (buff_step << 4) | DIAGONAL_R;
			}
		}
		head++;
	}
	head = 0;
	while (head < buff_tail)
	{
		if ((*(motion + head) & 0xf) == ADJUST)
		{
			if ((*(motion + head + 1) & 0xf) == LEFT)
			{
				uint16_t buff_step = *(motion + head) >> 4;
				*(motion + head) = (buff_step << 4) | ADJUST_L;
			}
			else if ((*(motion + head + 1) & 0xf) == RIGHT)
			{
				uint16_t buff_step = *(motion + head) >> 4;
				*(motion + head) = (buff_step << 4) | ADJUST_R;
			}
		}
		head++;
	}
}