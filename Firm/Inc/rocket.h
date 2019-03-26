/*
 * rocket.h
 *
 *  Created on: 2019/03/12
 *      Author: feunoir
 */

#ifndef ROCKET_H_
#define ROCKET_H_

#include "stm32f0xx_hal.h"

typedef enum {
	ROCKET_FALSE = 0x00,
	ROCKET_LAUNCHED = 0x01,
	ROCKET_ALLOWEDDEPLOY = 0x02,
	ROCKET_DEPLOYTIMERELAPSED = 0x08,
	ROCKET_ABLETODEPLOY_1STSTAGE = 0x10,
	ROCKET_ABLETODEPLOY_2NDSTAGE = 0x40
} Rocket_Status_t;

typedef enum {
	ACTUATOR_IDLE = 0x00,
	ACTUATOR_DRIVESERVO = 0x01,
	ACTUATOR_DRIVEAIRCYLINDER = 0x02
} Actuator_Status_t;

typedef struct {
	uint8_t rocket_status; // Rocket_Status_t�Q��
	uint8_t Actuator_Status_t;
} Rocket_Info_t;

void Rocket_Init(Rocket_Info_t info);
void Rocket_InitStatus(Rocket_Info_t info);

void Rocket_UpdateStatusLaunched(Rocket_Info_t *info);
void Rocket_UpdateStatusAllowDeploy(Rocket_Info_t *info);
void Rocket_UpdateStatusDeployTimerElapsed(Rocket_Info_t *info);
void Rocket_UpdateStatusAbleToDeploy_1stStage(Rocket_Info_t *info);
void Rocket_UpdateStatusAbleToDeploy_2ndStage(Rocket_Info_t *info);

Rocket_Status_t Rocket_isLaunched(Rocket_Info_t info);
Rocket_Status_t Rocket_isAllowedDeploy(Rocket_Info_t info);
Rocket_Status_t Rocket_isDeployTimerElapsed(Rocket_Info_t info);
Rocket_Status_t Rocket_isAbleToDeploy_1stStage(Rocket_Info_t info);
Rocket_Status_t Rocket_isAbleToDeploy_2ndStage(Rocket_Info_t info);

uint8_t Rocket_ReadStatus(Rocket_Info_t info, Rocket_Status_t selector);
void Rocket_SetStatus(Rocket_Info_t *info, Rocket_Status_t selector);
void Rocket_ResetStatus(Rocket_Info_t *info, Rocket_Status_t selector);

#endif /* ROCKET_H_ */
