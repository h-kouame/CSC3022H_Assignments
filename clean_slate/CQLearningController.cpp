/**
         (                                      
   (     )\ )                                   
 ( )\   (()/(   (    ) (        (        (  (   
 )((_)   /(_)) ))\( /( )(   (   )\  (    )\))(  
((_)_   (_))  /((_)(_)|()\  )\ |(_) )\ )((_))\  
 / _ \  | |  (_))((_)_ ((_)_(_/((_)_(_/( (()(_) 
| (_) | | |__/ -_) _` | '_| ' \)) | ' \)) _` |  
 \__\_\ |____\___\__,_|_| |_||_||_|_||_|\__, |  
                                        |___/   

Refer to Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
for a detailed discussion on Q Learning
*/
#include "CQLearningController.h"
#include "cdisccontroller.h"
#include "vector"
#include <climits>
#include <math.h>


CQLearningController::CQLearningController(HWND hwndMain):
	CDiscController(hwndMain),
	_grid_size_x(CParams::WindowWidth / CParams::iGridCellDim + 1),
	_grid_size_y(CParams::WindowHeight / CParams::iGridCellDim + 1)
{
}
/**
 The update method should allocate a Q table for each sweeper (this can
 be allocated in one shot - use an offset to store the tables one after the other)

 You can also use a boost multiarray if you wish
*/
void CQLearningController::InitializeLearningAlgorithm(void)
{
	//TODO
	for (int sweeper = 0; sweeper < CParams::iNumSweepers; ++sweeper) {
		std::vector<State> QTable;
		for (int i = 0; i < _grid_size_x*_grid_size_y; ++i) {
			//Action actions[4] = { {0, false}, {0, false}, { 0, false }, { 0, false } };
			State state = { {0, 0, 0, 0}, -1};
			QTable.push_back(state);
		}
		QTables.push_back(QTable);
	}
	//used to see use of rand() function
	//std::srand(std::time(0));  --already done in main
}
/**
 The immediate reward function. This computes a reward upon achieving the goal state of
 collecting all the mines on the field. It may also penalize movement to encourage exploring all directions and 
 of course for hitting supermines/rocks!
*/
double CQLearningController::R(uint x,uint y, uint sweeper_no){
	int cur_cell_index = ((y / CParams::iGridCellDim) * _grid_size_x) + x / CParams::iGridCellDim;
	int maxQ_ = maxQ(sweeper_no, cur_cell_index);
	//see if it's found a mine
	int GrabHit = m_vecSweepers[sweeper_no]->CheckForObject(m_vecObjects, CParams::dMineScale);
	////punish for finding nothing
	if (GrabHit < 0) return MOVEMENT_REWARD + GAMMA * maxQ_;
	//we have discovered a mine so increase give 100 reward
	else if (m_vecObjects[GrabHit]->getType() == CDiscCollisionObject::Mine) {
		++mine; return MINE_REWARD + GAMMA * maxQ_;
	}
	//we have hit a supermine so punish
	else if (m_vecObjects[GrabHit]->getType() == CDiscCollisionObject::SuperMine) {
		++mine; return SUPERMINE_REWARD + GAMMA * maxQ_;
	}
}
/**
The update method. Main loop body of our Q Learning implementation
See: Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
*/
bool CQLearningController::Update(void)
{
	//m_vecSweepers is the array of minesweepers
	//everything you need will be m_[something] ;)
	uint cDead = std::count_if(m_vecSweepers.begin(),
							   m_vecSweepers.end(),
						       [](CDiscMinesweeper * s)->bool{
								return s->isDead();
							   });
	if (cDead == CParams::iNumSweepers){
		printf("All dead ... skipping to next iteration\n");
		m_iTicks = CParams::iNumTicks;
	}

	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw){
		if (m_vecSweepers[sw]->isDead()) continue;
		/**
		Q-learning algorithm according to:
		Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
		*/
		//1:::Observe the current state:
		SVector2D<int> position = m_vecSweepers[sw]->Position();
		//TODO
		//2:::Select action with highest historic 
		int cell_index = ((position.y / CParams::iGridCellDim) * _grid_size_x) + position.x / CParams::iGridCellDim;
		int action = bestAction(sw, cell_index);
		m_vecSweepers[sw]->setRotation( (ROTATION_DIRECTION)action);
		QTables[sw][cell_index].last_action = action;
		//TODO
		//now call the parents update, so all the sweepers fulfill their chosen action
		//m_vecSweepers[sw]->Update(m_vecObjects); already done in CDiscController::Update();
	}
	
	CDiscController::Update(); //call the parent's class update. Do not delete this.
	
	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw){
		if (m_vecSweepers[sw]->isDead()) continue;
		//TODO:compute your indexes.. it may also be necessary to keep track of the previous state
		SVector2D<int> prev_pos = m_vecSweepers[sw]->PrevPosition();
		int prev_cell_index = ((prev_pos.y / CParams::iGridCellDim) * _grid_size_x) + prev_pos.x / CParams::iGridCellDim;
		//3:::Observe new state:
		SVector2D<int> position = m_vecSweepers[sw]->Position();
		//TODO
		//4:::Update _Q_s_a accordingly
		int action = QTables[sw][prev_cell_index].last_action;
		int reward_ = R(position.x, position.y, sw);
		//share info between sweepers: commented at the moment, to us uncomment for loop and replace sw by s in Q[sw...
		//for (int s = 0; s < CParams::iNumSweepers; ++s)
		QTables[sw][prev_cell_index].actions[action] = reward_;
	}
	return true;
}

int CQLearningController::maxQ(int sweeper, int cell_index) {
	int max = INT_MIN;
	for (int i = 0; i < 4; ++i) {
		max = max(QTables[sweeper][cell_index].actions[i], max);
	}
	//return 0 if all the neighbours have not yet been visited
	//if (max == INT_MIN) return 0;
	return max;
}

int CQLearningController::bestAction(int sweeper, int pre_index) {
	/*if (QTables[sweeper][pre_index].actions[0] == QTables[sweeper][pre_index].actions[1] &&
		QTables[sweeper][pre_index].actions[1] == QTables[sweeper][pre_index].actions[2] &&
		QTables[sweeper][pre_index].actions[2] == QTables[sweeper][pre_index].actions[3])
		return rand() % 4;*/
	int maxQ;
	int action = -1;
	maxQ = INT_MIN;
	for (int i = 0; i < 4; ++i) 
		maxQ = max(QTables[sweeper][pre_index].actions[i], maxQ);
	/*
	*Randomly choose between between one action if there are many maximums Q
	*/
	std::vector<int> max_indexes;
	for (int i = 0; i < 4; ++i)
		if (QTables[sweeper][pre_index].actions[i] == maxQ) max_indexes.push_back(i);
	return max_indexes[rand() % max_indexes.size()];
}

CQLearningController::~CQLearningController(void)
{
	//Nothing to be deallocated, I used std::vector
	
}