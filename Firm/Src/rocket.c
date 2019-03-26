/*
 * rocket.c
 *
 *  Created on: 2019/03/12
 *      Author: feunoir
 */

#include "rocket.h"
#include "stm32f0xx_hal.h"

void Rocket_Init(Rocket_Info_t info) {
	Rocket_InitStatus(info);
}

void Rocket_InitStatus(Rocket_Info_t info) {
	info.rocket_status = 0x00;
}

void Rocket_UpdateStatusLaunched(Rocket_Info_t *info){
	Rocket_SetStatus(info, ROCKET_LAUNCHED);
}

void Rocket_UpdateStatusAllowDeploy(Rocket_Info_t *info){
	Rocket_SetStatus(info, ROCKET_ALLOWEDDEPLOY);
}

void Rocket_UpdateStatusDeployTimerElapsed(Rocket_Info_t *info){
	Rocket_SetStatus(info, ROCKET_DEPLOYTIMERELAPSED);
}

void Rocket_UpdateStatusAbleToDeploy_1stStage(Rocket_Info_t *info){
	Rocket_SetStatus(info, ROCKET_ABLETODEPLOY_1STSTAGE);
}

void Rocket_UpdateStatusAbleToDeploy_2ndStage(Rocket_Info_t *info){
	Rocket_SetStatus(info, ROCKET_ABLETODEPLOY_2NDSTAGE);
}

Rocket_Status_t Rocket_isLaunched(Rocket_Info_t info){
	return Rocket_ReadStatus(info, ROCKET_LAUNCHED);
}

Rocket_Status_t Rocket_isAllowedDeploy(Rocket_Info_t info){
	return Rocket_ReadStatus(info, ROCKET_ALLOWEDDEPLOY);
}
Rocket_Status_t Rocket_isDeployTimerElapsed(Rocket_Info_t info){
	return Rocket_ReadStatus(info, ROCKET_DEPLOYTIMERELAPSED);
}
Rocket_Status_t Rocket_isAbleToDeploy_1stStage(Rocket_Info_t info){
	return Rocket_ReadStatus(info, ROCKET_ABLETODEPLOY_1STSTAGE);
}
Rocket_Status_t Rocket_isAbleToDeploy_2ndStage(Rocket_Info_t info){
	return Rocket_ReadStatus(info, ROCKET_ABLETODEPLOY_2NDSTAGE);
}

uint8_t Rocket_ReadStatus(Rocket_Info_t info, Rocket_Status_t selector) {
	return info.rocket_status & selector;
}

void Rocket_SetStatus(Rocket_Info_t *info, Rocket_Status_t selector) {
	info->rocket_status =  info->rocket_status|selector;
}

void Rocket_ResetStatus(Rocket_Info_t *info, Rocket_Status_t selector) {
	info->rocket_status &= ~selector;
}
