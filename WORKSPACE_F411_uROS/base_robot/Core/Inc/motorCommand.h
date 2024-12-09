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

int checkFrontObstacle(void);
int checkRearObstacle(void);
char process_command_data(char* buffer);
int process_vitess_data(char* buffer);
void get_vitess(char c);
void get_xy(char *buffer, float *result);

#endif /* INC_MOTORCOMMAND_H_ */
