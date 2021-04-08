/*
 * @Author: your name
 * @Date: 2021-04-01 23:32:00
 * @LastEditTime: 2021-04-05 19:03:11
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \ProjectSTM32F407c:\Users\42517\Documents\GitHub\ANO_LX_FC_F407\FcSrc\Ano_UWB.h
 */
#ifndef _ANO_UWB_H_
#define _ANO_UWB_H_

//==引用

//
// #include "Ano_Filter.h"
#include "Ano_Math.h"
// #include "Ano_Imu.h"
// #include "Ano_FcData.h"
//==定义
typedef struct
{
	u8 init_ok;
	u8 online;
	
	float ref_dir[2];
	float raw_data_loc[3];
	float raw_data_vel[3];	
	float w_dis_cm[3];
	float w_vel_cmps[3];
	float distance[4];
	float location_from_distance[3];
	float anthor_location[4][3];

}_uwb_data_st;


//==数据声明
extern _uwb_data_st uwb_data;


//==函数声明
void Ano_UWB_Get_Byte(u8 data);
void Ano_UWB_Get_Data_Task(u8 dT_ms);
void Ano_UWB_Data_Calcu_Task(u8 dT_ms);
void Anthor_location_init(float x0,float y0, float z0,float x1,float y1, float z1, float x2,float y2, float z2,float x3,float y3, float z3);
void ANO_UWB_Data_Receive_Anl(u8 UWB_RxBuffer,u8 UWB_data_cnt);



extern void Anthor_location_init(float x0,float y0, float z0,float x1,float y1, float z1, float x2,float y2, float z2,float x3,float y3, float z3);

void UWB_Location_Calculate(void);
extern void UWB_Location_Calculate(void);

#endif





