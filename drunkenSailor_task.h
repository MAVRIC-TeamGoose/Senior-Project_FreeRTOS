/*
 * Drunken Functions.h
 *
 *  Created on: 5/22/ 2015
 *      Author: Team Goose
 */

#ifndef DRUNKEN_WALK_H_
#define DRUNKEN_WALK_H_

// Random number generator
int genRand(int minSpeed,int maxSpeed);

// Left turn
void leftTurn(void);

// Right turn
void rightTurn(void);

// Back up
void backUp(void);

// wandering task
void startWandering(void);


// drunken walk algorithm
void drunken_Walk(void);

extern uint32_t DrunkenTaskInit(void);

#endif /* DUNKEN_WALK_H_ */
