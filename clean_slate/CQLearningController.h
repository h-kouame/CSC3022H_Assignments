#pragma once
#include "cdisccontroller.h"
#include "CParams.h"
#include "CDiscCollisionObject.h"
#include <cmath>
#include <vector>

typedef unsigned int uint;

#define GAMMA 0.9
#define SUPERMINE_REWARD -100
#define MINE_REWARD 10
#define MOVEMENT_REWARD -1

struct State {
	int actions[4];
	int last_action;
};

class CQLearningController :
	public CDiscController
{
private:
	uint _grid_size_x;
	uint _grid_size_y;
	int bestAction(int sweeper, int pre_index);
	int maxQ(int sweeper, int cell_index);
	int mine = 0;
	int super = 0;
public:
	CQLearningController(HWND hwndMain);
	virtual void InitializeLearningAlgorithm(void);
	double R(uint x, uint y, uint sweeper_no);
	virtual bool Update(void);
	virtual ~CQLearningController(void);
	std::vector<std::vector<State>> QTables;
};

