/*
 * @Author: your name
 * @Date: 2021-03-26 20:44:13
 * @LastEditTime: 2021-04-02 20:47:44
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: \ProjectSTM32F407c:\Users\42517\Documents\GitHub\ANO_LX_FC_F407\DriversMcu\STM32F407\Drivers\Drv_Uart.h
 */
#ifndef _USART_H
#define _USART_H

#include "SysConfig.h"
#include "Ano_UWB.h"

void DrvUart1Init(u32 br_num);
void Usart1_IRQ(void);
void DrvUart1SendBuf(unsigned char *DataToSend, u8 data_num);

extern u8 Rx_Buf[];
void DrvUart2Init(u32 br_num);
void Usart2_IRQ(void);
void DrvUart2SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUart3Init(u32 br_num);
void Usart3_IRQ(void);
void DrvUart3SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUart4Init(u32 br_num);
void Uart4_IRQ(void);
void DrvUart4SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUart5Init(u32 br_num);
void Uart5_IRQ(void);
void DrvUart5SendBuf(unsigned char *DataToSend, u8 data_num);

void DrvUartDataCheck(void);
#endif
