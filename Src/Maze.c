
#include "Maze.h"
#include "MazeCon.h"
#include "motion.h"
#include "adc.h"
#include "flash.h"

extern sensor_t sen_front;
unsigned char step[MAZE_SIZE][MAZE_SIZE];
unsigned char goal_size = 1;

void Maze_UpdatePosition(unsigned char *next_dir, pos_t *pos)
{
	if (*next_dir == FRONT)
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
	else if (*next_dir == LEFT)
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
	else if (*next_dir == RIGHT)
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
	else if (*next_dir == REAR)
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
	else if (*next_dir == PIVO_REAR)
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

	while (head != tail)
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

void Plan_Root(unsigned char motion[MAX_STEP], wallData_t *wall, unsigned char *tail)
{
	unsigned char flag_goal_is = FALSE;
	pos_t mypos;
	unsigned char buff_tail = 0;

	mypos.x = 0;
	mypos.y = 1;
	mypos.dir = NORTH;
	*(motion+buff_tail) = START;
	buff_tail++;

	while (flag_goal_is == FALSE)
	{
		unsigned char tmp;
		tmp = Maze_GetNextMotion(&mypos, wall);
		Maze_UpdatePosition(&tmp, &mypos);
		*(motion + buff_tail) = tmp;
		buff_tail++;
		if (step[mypos.x][mypos.y] == 0)
		{
			flag_goal_is = TRUE;
			*(motion + buff_tail) = GOAL;
			buff_tail++;
		}
	}

	*tail = buff_tail;
}

void Plan_Compress(uint8_t *a_qmotion, uint8_t *head, uint8_t *tail)
{
	uint8_t a_head = 0;
	uint8_t a_tail = *tail;
	uint8_t b_tail = 0;
	uint8_t motion;

	motion = *a_qmotion;
	a_head++;

	while (a_head != a_tail)
	{
		unsigned char buff = 0;
		switch (motion)
		{
		case START: //5
			motion = *(a_qmotion + a_head);
			a_head++;
			while (motion == FRONT && a_head != a_tail)
			{
				buff++;
				motion = *(a_qmotion + a_head);
				a_head++;
			}
			buff = (START << 4) | buff;
			*(a_qmotion + b_tail) = buff;
			b_tail++;
			break;
		case FRONT:
			while (motion == FRONT && a_head != a_tail)
			{
				buff++;
				motion = *(a_qmotion + a_head);
				a_head++;
			}
			if (motion == GOAL)
			{
				buff = GOAL << 4 | buff;
				*(a_qmotion + b_tail) = buff;
				b_tail++;
			}
			else
			{
				buff = FRONT << 4 | buff;
				*(a_qmotion + b_tail) = buff;
				b_tail++;
			}
			break;
		case LEFT:
			buff = LEFT << 4;
			*(a_qmotion + b_tail) = buff;
			b_tail++;
			motion = *(a_qmotion + a_head);
			a_head++;
			if (motion == GOAL)
			{
				buff = GOAL << 4;
				*(a_qmotion + b_tail) = buff;
				b_tail++;
			}
			break;
		case RIGHT:
			buff = RIGHT << 4;
			*(a_qmotion + b_tail) = buff;
			b_tail++;
			motion = *(a_qmotion + a_head);
			a_head++;
			if (motion == GOAL)
			{
				buff = GOAL << 4;
				*(a_qmotion + b_tail) = buff;
				b_tail++;
			}
			break;
		default:
			break;
		}
	}
	*tail = b_tail;
}

void Maze_Printf(void)
{
	wallData_t walldata;
	Flash_Load(start_address,(uint8_t*)&walldata,sizeof(wallData_t));
	printf("\r\n");
	for (unsigned char i = 0; i < MAZE_SIZE; i++)
	{
		printf("+");
		printf("---");
	}
	printf("+\r\n");
	for (unsigned char j = MAZE_SIZE; j > 0; j--)
	{
		for (unsigned char i = 0; i < MAZE_SIZE; i++)
		{
			if (walldata.vertical[i] & 0b1 << (j - 1))
			{
				printf("|");
			}
			else
			{
				printf(" ");
			}
			printf("   ");
		}
		printf("|");
		printf("\r\n");
		for (unsigned i = 0; i < MAZE_SIZE; i++)
		{
			printf("+");
			if (walldata.horizontal[j - 1] & 0b1 << i)
			{
				printf("---");
			}
			else
			{
				printf("   ");
			}
		}
		printf("+\r\n");
	}
}

/*
void Plan_Root(wallData_t *fastdata, uint8_t gool_x,uint8_t gool_y ,uint8_t *motion,uint8_t *head,uint8_t *tail){
	uint8_t flag_gool=0;
	Maze_UpdateStepMap(&flag_gool,gool_x,gool_y,fastdata);
	while (flag_gool==FALSE)
	{
		
	}
	
}
*/