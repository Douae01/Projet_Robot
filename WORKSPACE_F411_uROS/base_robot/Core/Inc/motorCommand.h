/*
 * MotorCommand.h
 */

#ifndef INC_MOTORCOMMAND_H_
#define INC_MOTORCOMMAND_H_

#include "main.h"


void motorCommand_Init(void);
void motorLeft_SetDuty(int);
void motorRight_SetDuty(int);

void onMoveForward(int duty);
void onMoveBackward(int duty);
void stopMoving(int duty);

#endif /* INC_MOTORCOMMAND_H_ */
