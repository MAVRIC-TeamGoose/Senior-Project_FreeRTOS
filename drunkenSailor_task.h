/*
 * Drunken Functions.h
 *
 *  Created on: 5/1/ 2015
 *  Author:Thinh- Team Goose
 */

#ifndef DRUNKEN_WALK_H_
#define DRUNKEN_WALK_H_

// Random number generator
int genRand(int min,int max);

// Left turn
void leftTurn();

// Right turn
void rightTurn();

// Back up
void backUp();

// wandering task
void startWandering();


// drunken walk algorithm
void drunken_Walk();

uint32_t DrunkenTaskInit(void);

#endif /* DUNKEN_WALK_H_ */
