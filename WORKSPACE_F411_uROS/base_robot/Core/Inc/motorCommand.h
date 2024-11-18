/*
 * MotorCommand.h
 */

#ifndef INC_MOTORCOMMAND_H_
#define INC_MOTORCOMMAND_H_

#include "main.h"
#include <stddef.h>


void motorCommand_Init(void);
void motorLeft_SetDuty(int);
void motorRight_SetDuty(int);

void onMoveForward(int index,int consigne);
void onMoveBackward(int index,int consigne);
void onMoveLeft(int index, int consigne);
void onMoveRight(int index, int consigne);
void stopMoving(int index);
int checkFrontObstacle(void);
int checkRearObstacle(void);
char process_command_data(char* buffer);
int process_vitess_data(char* buffer);
void get_vitess(char c);

typedef enum {
    MOVING_FORWARD,
    MOVING_BACKWARD,
    TURNING_LEFT,
    TURNING_RIGHT,
    STOPPED
} RobotState;

#endif /* INC_MOTORCOMMAND_H_ */
