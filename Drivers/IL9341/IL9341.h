/*
 * IL9341.h
 *
 *  Created on: 2021年10月9日
 *      Author: YIFAN
 */

#ifndef IL9341_IL9341_H_
#define IL9341_IL9341_H_

#include"main.h"
#define RES1  HAL_GPIO_WritePin(RES_GPIO_Port,RES_Pin,GPIO_PIN_SET)
#define RES0  HAL_GPIO_WritePin(RES_GPIO_Port,RES_Pin,GPIO_PIN_RESET)
#define DC1  HAL_GPIO_WritePin(DC_GPIO_Port,DC_Pin,GPIO_PIN_SET)
#define DC0  HAL_GPIO_WritePin(DC_GPIO_Port,DC_Pin,GPIO_PIN_RESET)
#define CS1  HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_SET)
#define CS0  HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_RESET)

SPI_HandleTypeDef hspi1;
void CMDWRITE(uint8_t cmd);
void DATAWRITE(uint8_t data);
void DATA16WRITE(uint16_t DATA);
void ADRSET(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCDINIT(void);
void LCDFILL(uint16_t color);
void DMAWRITE(uint8_t *data, uint16_t size);
#endif /* IL9341_IL9341_H_ */
