//==引用
#include "Ano_UWB.h"
//
// #include "Ano_Imu.h"
// #include "Ano_FcData.h"
//==定义
#define fc_sta flag

//==数据声明
_uwb_data_st uwb_data;


//*********************************************************************************************************
/**********************************************************************************************************
*函 数 名: Ano_UWB_Get_Byte
*功能说明: UWB获取字节
*参    数: 数据（1字节）
*返 回 值: 无
**********************************************************************************************************/
u8 UWB_RxBuffer[256],UWB_data_len = 0,UWB_Data_OK=0;

static u8 UWB_data_cnt;

float distance_to_zero[4];


void Ano_UWB_Get_Byte(u8 data)
{
static u8 _data_len = 0, _data_cnt = 0;
	static u8 rxstate = 0;

	//判断帧头是否满足匿名协议的0xAA
	if (rxstate == 0 && data == 0xAA)
	{
		rxstate = 1;
		UWB_RxBuffer[0] = data;
	}
	//判断是不是发送给本模块的数据或者是广播数据
	else if (rxstate == 1 && (data == HW_TYPE || data == HW_ALL))
	{
		rxstate = 2;
		UWB_RxBuffer[1] = data;
	}
	//接收帧CMD字节
	else if (rxstate == 2)
	{
		rxstate = 3;
		UWB_RxBuffer[2] = data;
	}
	//接收数据长度字节
	else if (rxstate == 3 && data < 250)
	{
		rxstate = 4;
		UWB_RxBuffer[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	//接收数据区
	else if (rxstate == 4 && _data_len > 0)
	{
		_data_len--;
		UWB_RxBuffer[4 + _data_cnt++] = data;
		if (_data_len == 0)
			rxstate = 5;
		UWB_data_len = data;
	}
	//接收校验字节1
	else if (rxstate == 5)
	{
		rxstate = 6;
		UWB_RxBuffer[4 + _data_cnt++] = data;
	}
	//接收校验字节2，表示一帧数据接收完毕，调用数据解析函数
	else if (rxstate == 6)
	{
		rxstate = 0;
		UWB_RxBuffer[4 + _data_cnt] = data;
		UWB_data_cnt = _data_cnt + 5;
		//ano_UWB_data_ok = 1;
		Ano_UWB_Get_Data_Task(30);
		UWB_Location_Calculate();
	}
	else
	{
		rxstate = 0;
	}
}

/**********************************************************************************************************
*函 数 名: Ano_UWB_Get_Data_Task
*功能说明: UWB数据获取任务
*参    数: 周期（毫秒）
*返 回 值: 无
**********************************************************************************************************/

void Anthor_location_init(float x0,float y0, float z0,float x1,float y1, float z1, float x2,float y2, float z2,float x3,float y3, float z3)
{
	uwb_data.anthor_location[0][0]= x0;
	uwb_data.anthor_location[0][1]= y0;
	uwb_data.anthor_location[0][2]= z0;
	uwb_data.anthor_location[1][0]= x1;
	uwb_data.anthor_location[1][1]= y1;
	uwb_data.anthor_location[1][2]= z1;
	uwb_data.anthor_location[2][0]= x2;
	uwb_data.anthor_location[2][1]= y2;
	uwb_data.anthor_location[2][2]= z2;
	uwb_data.anthor_location[3][0]= x3;
	uwb_data.anthor_location[3][1]= y3;
	uwb_data.anthor_location[3][2]= z3;
	distance_to_zero [0] = x0*x0 + y0*y0 + z0*z0;
	distance_to_zero [1] = x1*x1 + y1*y1 + z1*z1;
	distance_to_zero [2] = x2*x2 + y2*y2 + z2*z2;
	distance_to_zero [3] = x3*x3 + y3*y3 + z3*z3;
}


//note
void UWB_Location_Calculate(void)
{
	uwb_data.location_from_distance[0] = -(uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[2][2] - uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[1][2] - uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[2][2] + uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[0][2] + uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[1][2] - uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[0][2] - uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[3][2] + uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[1][2] + uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[3][2] - uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[0][2] - uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[1][2] + uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[0][2] + uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[3][2] - uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[2][2] - uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[3][2] + uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[0][2] + uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[2][2] - uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[0][2] - uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[3][2] + uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[2][2] + uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[3][2] - uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[1][2] - uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[2][2] + uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[1][2] - distance_to_zero [0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[2][2] + distance_to_zero [0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[1][2] + distance_to_zero [1]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[2][2] - distance_to_zero [1]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[0][2] - distance_to_zero [2]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[1][2] + distance_to_zero [2]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[0][2] + distance_to_zero [0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[3][2] - distance_to_zero [0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[1][2] - distance_to_zero [1]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[3][2] + distance_to_zero [1]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[0][2] + distance_to_zero [3]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[1][2] - distance_to_zero [3]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[0][2] - distance_to_zero [0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[3][2] + distance_to_zero [0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[2][2] + distance_to_zero [2]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[3][2] - distance_to_zero [2]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[0][2] - distance_to_zero [3]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[2][2] + distance_to_zero [3]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[0][2] + distance_to_zero [1]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[3][2] - distance_to_zero [1]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[2][2] - distance_to_zero [2]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[3][2] + distance_to_zero [2]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[1][2] + distance_to_zero [3]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[2][2] - distance_to_zero [3]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[1][2])/(2*(uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[2][2] - uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[1][2] - uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[2][2] + uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[0][2] + uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[1][2] - uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[0][2] - uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[3][2] + uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[1][2] + uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[3][2] - uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[0][2] - uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[1][2] + uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[0][2] + uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[3][2] - uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[2][2] - uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[3][2] + uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[0][2] + uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[2][2] - uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[0][2] - uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[3][2] + uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[2][2] + uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[3][2] - uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[1][2] - uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[2][2] + uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[1][2]));
	uwb_data.location_from_distance[1] = (uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][2] - uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][2] - uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][2] + uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][2] + uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][2] - uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][2] - uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][2] + uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][2] + uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][2] - uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][2] - uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][2] + uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][2] + uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][2] - uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][2] - uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][2] + uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][2] + uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][2] - uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][2] - uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][2] + uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][2] + uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][2] - uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][2] - uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][2] + uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][2] - distance_to_zero [0]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][2] + distance_to_zero [0]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][2] + distance_to_zero [1]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][2] - distance_to_zero [1]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][2] - distance_to_zero [2]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][2] + distance_to_zero [2]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][2] + distance_to_zero [0]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][2] - distance_to_zero [0]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][2] - distance_to_zero [1]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][2] + distance_to_zero [1]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][2] + distance_to_zero [3]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][2] - distance_to_zero [3]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][2] - distance_to_zero [0]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][2] + distance_to_zero [0]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][2] + distance_to_zero [2]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][2] - distance_to_zero [2]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][2] - distance_to_zero [3]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][2] + distance_to_zero [3]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][2] + distance_to_zero [1]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][2] - distance_to_zero [1]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][2] - distance_to_zero [2]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][2] + distance_to_zero [2]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][2] + distance_to_zero [3]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][2] - distance_to_zero [3]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][2])/(2*(uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[2][2] - uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[1][2] - uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[2][2] + uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[0][2] + uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[1][2] - uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[0][2] - uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[3][2] + uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[1][2] + uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[3][2] - uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[0][2] - uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[1][2] + uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[0][2] + uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[3][2] - uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[2][2] - uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[3][2] + uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[0][2] + uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[2][2] - uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[0][2] - uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[3][2] + uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[2][2] + uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[3][2] - uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[1][2] - uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[2][2] + uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[1][2]));
	uwb_data.location_from_distance[2] = -(uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][1] - uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][1] - uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][1] + uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][1] + uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][1] - uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][1] - uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][1] + uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][1] + uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][1] - uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][1] - uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][1] + uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][1] + uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][1] - uwb_data.distance[0]*uwb_data.distance[0]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][1] - uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][1] + uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][1] + uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][1] - uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][1] - uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][1] + uwb_data.distance[1]*uwb_data.distance[1]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][1] + uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][1] - uwb_data.distance[2]*uwb_data.distance[2]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][1] - uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][1] + uwb_data.distance[3]*uwb_data.distance[3]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][1] - distance_to_zero [0]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][1] + distance_to_zero [0]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][1] + distance_to_zero [1]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][1] - distance_to_zero [1]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][1] - distance_to_zero [2]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][1] + distance_to_zero [2]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][1] + distance_to_zero [0]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][1] - distance_to_zero [0]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][1] - distance_to_zero [1]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][1] + distance_to_zero [1]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][1] + distance_to_zero [3]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][1] - distance_to_zero [3]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][1] - distance_to_zero [0]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][1] + distance_to_zero [0]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][1] + distance_to_zero [2]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][1] - distance_to_zero [2]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][1] - distance_to_zero [3]*uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][1] + distance_to_zero [3]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][1] + distance_to_zero [1]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][1] - distance_to_zero [1]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][1] - distance_to_zero [2]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][1] + distance_to_zero [2]*uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][1] + distance_to_zero [3]*uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][1] - distance_to_zero [3]*uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][1])/(2*(uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[2][2] - uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[1][2] - uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[2][2] + uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[0][2] + uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[1][2] - uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[0][2] - uwb_data.anthor_location[0][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[3][2] + uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[1][2] + uwb_data.anthor_location[1][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[3][2] - uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[0][2] - uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[1][2] + uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[0][2] + uwb_data.anthor_location[0][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[3][2] - uwb_data.anthor_location[0][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[2][2] - uwb_data.anthor_location[2][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[3][2] + uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[0][2] + uwb_data.anthor_location[3][0]*uwb_data.anthor_location[0][1]*uwb_data.anthor_location[2][2] - uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[0][2] - uwb_data.anthor_location[1][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[3][2] + uwb_data.anthor_location[1][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[2][2] + uwb_data.anthor_location[2][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[3][2] - uwb_data.anthor_location[2][0]*uwb_data.anthor_location[3][1]*uwb_data.anthor_location[1][2] - uwb_data.anthor_location[3][0]*uwb_data.anthor_location[1][1]*uwb_data.anthor_location[2][2] + uwb_data.anthor_location[3][0]*uwb_data.anthor_location[2][1]*uwb_data.anthor_location[1][2]));
}


u8 i =0;

static u16 uwb_check_time;
void Ano_UWB_Get_Data_Task(u8 UWB_ms)
{
	if(1)
	//if(UWB_Data_OK)
	{
		u8 sum = 0;
		for( i=0;i<(UWB_data_len-1);i++)
			sum += *(UWB_RxBuffer+i);
		if(!(sum==*(UWB_RxBuffer+UWB_data_len-1)))		return;		//判断sum
		
		uint8_t temp=0;
 		//////////////////
		if(*(UWB_RxBuffer+3)==0x31)			//距离信息
		{
			uwb_data.distance[0] = (float)(s16)((*(UWB_RxBuffer+5)<<8)|*(UWB_RxBuffer+6)) / 100;
			uwb_data.distance[1] = (float)(s16)((*(UWB_RxBuffer+7)<<8)|*(UWB_RxBuffer+8)) / 100;
			uwb_data.distance[2] = (float)(s16)((*(UWB_RxBuffer+9)<<8)|*(UWB_RxBuffer+10)) / 100;
			uwb_data.distance[3] = (float)(s16)((*(UWB_RxBuffer+11)<<8)|*(UWB_RxBuffer+12)) / 100;
		}
		else if(*(UWB_RxBuffer+3)==0x32)			//位置信息
		{
			uwb_data.raw_data_loc[1] = -(float)(s16)((*(UWB_RxBuffer+6)<<8)|*(UWB_RxBuffer+7)) / 100;
			uwb_data.raw_data_loc[0] =  (float)(s16)((*(UWB_RxBuffer+8)<<8)|*(UWB_RxBuffer+9)) / 100;
			uwb_data.raw_data_loc[2] =  (float)(s16)((*(UWB_RxBuffer+10)<<8)|*(UWB_RxBuffer+11)) / 100;
			uwb_data.raw_data_vel[1] = -(float)(s16)((*(UWB_RxBuffer+12)<<8)|*(UWB_RxBuffer+13)) / 100;
			uwb_data.raw_data_vel[0] =  (float)(s16)((*(UWB_RxBuffer+14)<<8)|*(UWB_RxBuffer+15)) / 100;
			uwb_data.raw_data_vel[2] =  (float)(s16)((*(UWB_RxBuffer+16)<<8)|*(UWB_RxBuffer+17)) / 100;
		}
		//
		uwb_check_time = 0;
	}
	//
	if(uwb_check_time <1000)
	{
		uwb_check_time += UWB_ms;
		uwb_data.online = 1;
	}
	else
	{
		uwb_data.online = 0;
	}
	
}

/**********************************************************************************************************
*函 数 名: Ano_UWB_Data_Calcu_Task
*功能说明: UWB数据计算任务
*参    数: 周期（毫秒）
*返 回 值: 无
**********************************************************************************************************/
// void Ano_UWB_Data_Calcu_Task(u8 UWB_ms)
// {
// 	//解锁前，记录当前机体X轴正方向在水平面投影为UWB坐标X轴正方向
// 	//要求解锁前，飞机放正，机体X轴正方向对准UWB坐标轴X轴正方向
// 	//记录参考方向
// 	if(!fc_sta.unlock_sta)
// 	{
// 		//
// 		uwb_data.init_ok = 0;
// 		//
// 		uwb_data.ref_dir[X] = imu_data.hx_vec[X];
// 		uwb_data.ref_dir[Y] = imu_data.hx_vec[Y];

// 	}
// 	//
// 	else
// 	{
// 		//
// 		uwb_data.init_ok = 1;
// 	}
// 	//参考方向转世界坐标（这里等同于地理坐标）
// 	h2w_2d_trans(uwb_data.raw_data_loc,uwb_data.ref_dir,uwb_data.w_dis_cm);
// 	//计算速度
// 	h2w_2d_trans(uwb_data.raw_data_vel,uwb_data.ref_dir,uwb_data.w_vel_cmps);
	
// }
