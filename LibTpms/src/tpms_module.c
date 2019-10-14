
#include "LibTpms.h"
#include "lw_type.h"
#include "Tpms_inner.h"

#include <string.h>
#include <math.h>
#include <stdint.h>


/*
1、对于测试路段，学习时间为20分钟
2、测试路段为来回，激活阶段可以不过滤转弯
*/

static TPMS_MANAGER_S gstTpmsManager;

static INDIRECT_TPMS_DATA_S gpstTpmsData;



static u8 gAllWheelErrorFlag = 0;
//static u8 gAllWheelErrorTimer = 0;



/*为方便测试设置的速度阈值*/
static u16 gTpmsSpeedMax = 10000;
static u16 gTpmsSpeedMin = 3500;
static u8 gCalibrateCmd = 0;

float rtData[4] = {0};
    
#if USE_ACCE
#define ACCE_THRESHOLD          0.04
#define GYRO_THRESHOLD          10

extern float g_acce_init_val[3];

void tpms_AcceInit(void)
{
	float acce_angle[2][3];

	osDelay(5000);
	GyroAndAcce_Init();//初始化陀螺仪+加速度传感器
	
	osDelay(1000);

	GyroAndAcce_Read_Init_Angle(acce_angle[0]);/*判断拖车的角度每次重启时都要读*/

	if (param.acce_flag==1)
	{
		memcpy(g_acce_init_angle, param.AngleInit, 12);
		//memcpy(acce_angle[0], g_acce_init_angle, 12);
		g_acce_init_val[0] = cos(g_acce_init_angle[0]*3.14/180);
		g_acce_init_val[1] = cos(g_acce_init_angle[1]*3.14/180);
		g_acce_init_val[2] = cos(g_acce_init_angle[2]*3.14/180);
	}
	else
	{
		/*判断拖车的角度每次重启时都要读*/
		GyroAndAcce_Read_Init_Angle(acce_angle[0]);
		osDelay(1000);
		memcpy(g_acce_init_angle, acce_angle[0], 12);
		
		GyroAndAcce_Read_Init_gValue(g_acce_init_val);/*用于行驶状态下的偏移*/
	}

	GyroAndAcce_print_dir(g_acce_init_angle);

	osDelay(1000);
	print_info("init acce angle: %.2f, %.2f, %.2f\r\n", acce_angle[0][0], acce_angle[0][1], acce_angle[0][2]);
	
}

u8 tpms_AcceRead(void)
{
    float acce_xyz[3];
	float dps_xyz[3];
	int ret1, ret2;

		
	ret1 = GyroAndAcce_Read_Value_dps(dps_xyz);
	ret2 = GyroAndAcce_Read_Value_g(acce_xyz);
	if (ret2 < 0 || ret1 < 0)
	{
	    print_info("get acce or dps error\r\n");
        return 0;
	}
	else
	{
		acce_xyz[0] -= g_acce_init_val[0];//减去初始分量
		acce_xyz[1] -= g_acce_init_val[1];
		acce_xyz[2] -= g_acce_init_val[2];
    }

    if ((acce_xyz[0]*acce_xyz[0]+acce_xyz[1]*acce_xyz[1]+acce_xyz[2]*acce_xyz[2]) > ACCE_THRESHOLD)
    {
        print_info("acce\r\n");
        return 1;
    }

    if (fabs(dps_xyz[0]) > GYRO_THRESHOLD ||
        fabs(dps_xyz[1]) > GYRO_THRESHOLD ||
        fabs(dps_xyz[2]) > GYRO_THRESHOLD )
    {
        print_info("turn\r\n");
        return 1;
    }

    return 0;
}
#endif

void tpms_StopCalibrate(void)
{
    gCalibrateCmd = 1;
}

/*for test only*/
void tpms_SetSpeedRange(u16 min, u16 max)
{
    gTpmsSpeedMax = max;
    gTpmsSpeedMin = min;
}


/******************************************************************************
	描    述:   胎压标定
	输入参数:   speed:5个车速
	返 回 值:   无
	注意事项:   无
******************************************************************************/
 static void tpms_Calibrate(u16 *speed)
{
    static float speedTotal = 0;
    static float wheelSpeed[4] = {0};
    float avg_speed;
    float percent[4] = {0};
    u8 i;
    //static float tpms_offset[4] = {0};

    static u16 dataCount = 0;   /*学习点数*/

    avg_speed = (speed[1] + speed[2] + speed[3] + speed[4])/4;
    
    percent[0] = ((float)(speed[1] - avg_speed))/avg_speed;
    percent[1] = ((float)(speed[2] - avg_speed))/avg_speed;
    percent[2] = ((float)(speed[3] - avg_speed))/avg_speed;
    percent[3] = ((float)(speed[4] - avg_speed))/avg_speed;

    if ((percent[0]<-0.002 &&percent[2]<-0.002) ||(percent[0]>0.002 && percent[2]>0.002)
    ||(percent[1]<-0.002 && percent[3]<-0.002) ||(percent[1]>0.002 && percent[3]>0.002))
    {
        //printf("LibTpms: turn left or right\r\n");        
        return;
    }

    /*异常数据*/
    if ((fabs(percent[0])>0.02) || (fabs(percent[1])>0.02) || (fabs(percent[2])>0.02) || (fabs(percent[3])>0.02))
    {
        //printf("LibTpms: abnormal data\r\n");
        return;
    }

    if (speed[0]<gTpmsSpeedMin || speed[0]>gTpmsSpeedMax)
    {
        return;
    }

    if (speed[0]<3500)
    {
        return;
    }

    dataCount++;
    speedTotal += speed[0];

    wheelSpeed[0] += speed[1];
    wheelSpeed[1] += speed[2];
    wheelSpeed[2] += speed[3];
    wheelSpeed[3] += speed[4];

    gstTpmsManager.Tpms_Print("LibTpms: learn count:%d\r\n", dataCount);
    
    if (dataCount>=600 || gCalibrateCmd==1)
    {
        gstTpmsManager.Tpms_Print("LibTpms: speed: %f, %f, %f, %f, %f\r\n", speedTotal, wheelSpeed[0], wheelSpeed[1], wheelSpeed[2],wheelSpeed[3]);
        for (i=0; i<4; i++)
        {
            gstTpmsManager.tpmsOffset[i] = ((float)(wheelSpeed[i] - speedTotal))/speedTotal;
        }
        gstTpmsManager.learnFlag = 1;
        gstTpmsManager.Tpms_Print("LibTpms: learn complete: %f %f %f %f\r\n", 
            gstTpmsManager.tpmsOffset[0], gstTpmsManager.tpmsOffset[1],
            gstTpmsManager.tpmsOffset[2], gstTpmsManager.tpmsOffset[3]);

        gstTpmsManager.Tpms_SetLearnFlag(1, gstTpmsManager.tpmsOffset);
        dataCount = 0;
        if (gCalibrateCmd==1)
        {
            gCalibrateCmd = 0;
        }
    }
    
}

/*

*/
/******************************************************************************
	描    述:   计算胎压
	输入参数:   speed:车速
	返 回 值:   无
	注意事项:   1、速度小于35码过滤
                2、不判断上下坡和转弯
                3、每次计算TPMS_DATA_COUNT个点数
                4、连续两次判断异常并且异常轮胎位置一致则上报异常
                5、上报异常后，除非清零，否则不再判断
******************************************************************************/
static void tpms_Algorithm(u16 *speed)
{
    u8 i, j;
    float rtTpmsData[4] = {0};
    float speedTotal[5];
    u8 abnw = 0;
    //static u8 abnwCount = 0;
    //static u8 abnwBefore = 0;
    //float rtData[4] = {0};    /*将此数组作为全局变量*/

    if (speed[0]<gTpmsSpeedMin || speed[0]>gTpmsSpeedMax)
    {
        return;
    }

    memset(speedTotal, 0, sizeof(speedTotal));    

    
    gpstTpmsData.dataCount++;
    gpstTpmsData.tpmsData[0][gpstTpmsData.dataPointer] = speed[0];
    gpstTpmsData.tpmsData[1][gpstTpmsData.dataPointer] = speed[1];
    gpstTpmsData.tpmsData[2][gpstTpmsData.dataPointer] = speed[2];
    gpstTpmsData.tpmsData[3][gpstTpmsData.dataPointer] = speed[3];
    gpstTpmsData.tpmsData[4][gpstTpmsData.dataPointer] = speed[4];
    
    gpstTpmsData.dataPointer++;
    if (gpstTpmsData.dataPointer == TPMS_DATA_COUNT)
    {
        gpstTpmsData.dataPointer = 0;
    }
    gstTpmsManager.Tpms_Print("LibTpms: count is %d\r\n", gpstTpmsData.dataCount);

    if (gpstTpmsData.dataCount >= TPMS_DATA_COUNT)
    {
        gpstTpmsData.dataCount -= 1;
        for (j=0; j<5; j++)
        {
            for (i=0; i<TPMS_DATA_COUNT; i++)
            {
                speedTotal[j] += gpstTpmsData.tpmsData[j][i];
            }
        }
        gstTpmsManager.Tpms_Print("LibTpms: speed total: %f, %f, %f, %f, %f\r\n", speedTotal[0], speedTotal[1],speedTotal[2],speedTotal[3],speedTotal[4]);
        for (i=0; i<4; i++)
        {
            rtTpmsData[i] = ((float)(speedTotal[i+1]-speedTotal[0]))/speedTotal[0];
            rtData[i] = rtTpmsData[i]-gstTpmsManager.tpmsOffset[i];
            gstTpmsManager.Tpms_Print("LibTpms: i is %d, offset is %f\r\n", i, rtData[i]);
        }

        /*相差大时可以放宽绝对数值的要求*/
        /*需要过滤车速的影响,车速高时，rtData[i]可能是负值*/
        for (i=0; i<4; i++)
        {
            /*车速每增加20，值大约减少0.0005*/
            rtData[i] -= 0.0005-(speedTotal[0]/TPMS_DATA_COUNT-4000)/2000*0.0005;
        }

        if (speedTotal[0]/TPMS_DATA_COUNT< 9000)
        {

        /*算法2: 相互比较超出阈值。前轮与前轮比较，后轮与后轮比较*/
        if (rtData[0]<0.0003 && rtData[1]>-0.0003)
        {
            if (rtData[1]-rtData[0]>0.0023)
            {
                abnw |= 1<<2;
            }
        }

        if (rtData[1]<0.0003 && rtData[0]>-0.0003)
        {
            if (rtData[0]-rtData[1]>0.0023)
            {
                abnw |= 1<<3;
            }
        }

        if (rtData[3]<0.0003 && rtData[2]>-0.0003)
        {
            if (rtData[2]-rtData[3]>0.0023)
            {
                abnw |= 1<<1;
            }
        }

        if (rtData[2]<0.0003 && rtData[3]>-0.0003)
        {
            if (rtData[3]-rtData[2]>0.0023)
            {
                abnw |= 1<<0;
            }
        }

        /*同轴轮胎异常的情况:后面两个轮胎同时异常的情况*/
        if (rtData[2]>-0.0003 && rtData[3]>-0.0003)
        {
            if (rtData[0]<0.0003 && rtData[1]<0.0003)
            {
                if (rtData[2]-rtData[0] > 0.0023 &&
                    rtData[3]-rtData[1] > 0.0023)
                {
                    abnw = 3;
                }
                
            }
            /*3个轮子和两个轮子异常的阈值不同*/
            else if (rtData[0]<0.0003 && rtData[1]>-0.0003)
            {
                if (rtData[2]-rtData[0] > 0.0020 &&
                    rtData[3]-rtData[0] > 0.0020 &&
                    rtData[1]-rtData[0] > 0.0020)
                {
                    abnw = 0x7;
                }
            }
            else if (rtData[0]>-0.0003 && rtData[1]<0.0003)
            {
                if (rtData[2]-rtData[1] > 0.0020 && 
                    rtData[3]-rtData[1] > 0.0020 &&
                    rtData[0]-rtData[1] > 0.0020)
                {
                    abnw = 0xb;
                }
            }
        }

        /*前面两个轮子同时异常的情况阈值可能需要调整*/
        if (rtData[2]<-0.0025 && rtData[3]<-0.0025)
        {
            if (rtData[0]-rtData[2]>0.0025 && rtData[1]-rtData[3]>0.0025)
            {
                abnw = 0xc;
            }
        }

        if (rtData[0]<0.0003 && rtData[1]<0.0003 && rtData[2]<0.0003 && rtData[3]<-0.0025)
        {
            if (rtData[0]-rtData[3]>0.0020 
                && rtData[1]-rtData[3]>0.0020
                && rtData[2]-rtData[3]>0.0020)
            {
                abnw = 0xe;
            }
        }

        if (rtData[0]<0.0003 && rtData[1]<0.0003 && rtData[3]<0.0003 && rtData[2]<-0.0025)
        {
            if (rtData[0]-rtData[2]>0.0020 
                && rtData[1]-rtData[2]>0.0020
                && rtData[3]-rtData[2]>0.0020)
            {
                abnw = 0xd;
            }
        }
        }
        else
        {
            /*100码以上使用宽松的算法*/

            /*1、一个轮子*/
            if (rtData[0]>-0.0003 && rtData[1]<0.0003 && rtData[2]<0.0003 && rtData[3]<0.0003 && 
                (rtData[0]-rtData[1]>0.0023 || rtData[0]-rtData[2]>0.0023))
            {
                abnw = 0x08;
            }

            if (rtData[1]>-0.0003 && rtData[0]<0.0003 && rtData[2]<0.0003 && rtData[3]<0.0003 && 
                (rtData[1]-rtData[0]>0.0023 || rtData[1]-rtData[3]>0.0023))
            {
                abnw = 0x04;
            }

            if (rtData[2]>-0.0003 && rtData[0]<0.0003 && rtData[1]<0.0003 && rtData[3]<0.0003 && 
                (rtData[2]-rtData[3]>0.0023 || rtData[2]-rtData[0]>0.0023))
            {
                abnw = 0x02;
            }

            if (rtData[3]>-0.0003 && rtData[0]<0.0003 && rtData[1]<0.0003 && rtData[2]<0.0003 && 
                (rtData[3]-rtData[2]>0.0023 || rtData[3]-rtData[1]>0.0023))
            {
                abnw = 0x01;
            }

            /*2、不同轴两个轮子*/

            if (rtData[0]>-0.0003 && rtData[1]<0.0003 && rtData[2]>-0.0003 && rtData[3]<0.0003)
            {
                if (rtData[0]-rtData[1]>0.0023 && rtData[2]-rtData[3]>0.0023)
                {
                    abnw = 0x0a;
                }
            }

            if (rtData[1]>-0.0003 && rtData[0]<0.0003 && rtData[3]>-0.0003 && rtData[2]<0.0003)
            {
                if (rtData[1]-rtData[0]>0.0023 && rtData[3]-rtData[2]>0.0023)
                {
                    abnw = 0x05;
                }
            }

            if (rtData[0]>-0.0003 && rtData[1]<0.0003 && rtData[3]>-0.0003 && rtData[2]<0.0003)
            {
                if ((rtData[0]-rtData[1]>0.0023 && rtData[3]-rtData[2]>0.0023) ||
                    (rtData[0]-rtData[1]>0.0023 && rtData[3]-rtData[1]>0.0023) ||
                    (rtData[0]-rtData[2]>0.0023 && rtData[3]-rtData[2]>0.0023) ||
                    (rtData[0]-rtData[2]>0.0023 && rtData[3]-rtData[1]>0.0023))
                {
                    abnw = 0x09;
                }
            }

            if (rtData[1]>-0.0003 && rtData[0]<0.0003 && rtData[2]>-0.0003 && rtData[3]<0.0003)
            {
                if ((rtData[1]-rtData[0]>0.0023 && rtData[2]-rtData[3]>0.0023) ||
                    (rtData[1]-rtData[0]>0.0023 && rtData[2]-rtData[0]>0.0023) ||
                    (rtData[1]-rtData[3]>0.0023 && rtData[2]-rtData[3]>0.0023) ||
                    (rtData[1]-rtData[3]>0.0023 && rtData[2]-rtData[0]>0.0023))
                {
                    abnw = 0x06;
                }
            }
            
            
            /*算法2: 相互比较超出阈值。前轮与前论比较，后轮与后轮比较*/
            

            /*同轴轮胎异常的情况:后面两个轮胎同时异常的情况*/
            if (rtData[2]>-0.0003 && rtData[3]>-0.0003)
            {
                if (rtData[0]<0.0003 && rtData[1]<0.0003)
                {
                    if (rtData[2]-rtData[0] > 0.0023 &&
                        rtData[3]-rtData[1] > 0.0023)
                    {
                        abnw = 3;
                    }
                    
                }
                /*3个轮子和两个轮子异常的阈值不同*/
                else if (rtData[0]<0.0003 && rtData[1]>-0.0003)
                {
                    if (rtData[2]-rtData[0] > 0.0020 &&
                        rtData[3]-rtData[0] > 0.0020 &&
                        rtData[1]-rtData[0] > 0.0020)
                    {
                        abnw = 0x7;
                    }
                }
                else if (rtData[0]>-0.0003 && rtData[1]<0.0003)
                {
                    if (rtData[2]-rtData[1] > 0.0020 && 
                        rtData[3]-rtData[1] > 0.0020 &&
                        rtData[0]-rtData[1] > 0.0020)
                    {
                        abnw = 0xb;
                    }
                }
            }

            /*前面两个轮子同时异常的情况阈值可能需要调整*/
            if (rtData[2]<-0.0023 && rtData[3]<-0.0023)
            {
                if (rtData[0]-rtData[2]>0.0023 && rtData[1]-rtData[3]>0.0023)
                {
                    abnw = 0xc;
                }
            }

            if (rtData[0]<0.0003 && rtData[1]<0.0003 && rtData[2]<0.0003 && rtData[3]<-0.0023)
            {
                if (rtData[0]-rtData[3]>0.0020 
                    && rtData[1]-rtData[3]>0.0020
                    && rtData[2]-rtData[3]>0.0020)
                {
                    abnw = 0xe;
                }
            }

            if (rtData[0]<0.0003 && rtData[1]<0.0003 && rtData[3]<0.0003 && rtData[2]<-0.0023)
            {
                if (rtData[0]-rtData[2]>0.0020 
                    && rtData[1]-rtData[2]>0.0020
                    && rtData[3]-rtData[2]>0.0020)
                {
                    abnw = 0xd;
                }
            }
        }

        /*多个轮子胎压异常容易误报、少报*/
        switch (abnw)
        {
            case 1:
                if (rtData[0]>0.0003 ||
                    rtData[1]>0.0003 ||
                    rtData[2]>0.0003 ||
                    rtData[1]-rtData[0]>0.0016 ||
                    rtData[0]-rtData[1]>0.0016 ||
                    rtData[0]-rtData[2]>0.0016 ||
                    rtData[1]-rtData[2]>0.0016)
                {
                    abnw = 0;
                }
                break;
            case 2:
                if (rtData[0]>0.0003 ||
                    rtData[1]>0.0003 ||
                    rtData[3]>0.0003 ||
                    rtData[1]-rtData[0]>0.0016 ||
                    rtData[0]-rtData[1]>0.0016 ||
                    rtData[0]-rtData[3]>0.0016 ||
                    rtData[1]-rtData[3]>0.0016)
                {
                    abnw = 0;
                }
                break;
            case 3:
                if (rtData[0]>0 ||
                    rtData[1]>0)
                {
                    abnw = 0;
                }
                break;
            
            case 4:
                if (rtData[0]>0.0003 ||
                    rtData[2]>0.0003 ||
                    rtData[3]>0.0003 ||
                    rtData[2]-rtData[3]>0.0016 ||
                    rtData[3]-rtData[2]>0.0016)
                {
                    abnw = 0;
                }
                break;
            case 5:
                if (rtData[0]>0.0003 ||
                    rtData[2]>0.0003)
                {
                    abnw = 0;
                }
                break;
            case 6:
                if (rtData[0]>0.0003 ||
                    rtData[3]>0.0003)
                {
                    abnw = 0;
                }
                break;
            case 7:
                if ((rtData[0]) > 0.0003)
                {
                    abnw = 0;
                }
                break;
            case 8:
                if (rtData[1]>0.0003 ||
                    rtData[2]>0.0003 ||
                    rtData[3]>0.0003 ||
                    rtData[2]-rtData[3]>0.0016 ||
                    rtData[3]-rtData[2]>0.0016)
                {
                    abnw = 0;
                }
                break;
            case 9:
                if (rtData[1]>0.0003 ||
                    rtData[2]>0.0003)
                {
                    abnw = 0;
                }
                break;
            case 0x0a:
                if (rtData[1]>0.0003 ||
                    rtData[3]>0.0003)
                {
                    abnw = 0;
                }
                break;
            case 0x0b:
                if (rtData[1]>0.0003)
                {
                    abnw = 0;
                }
                break;
        }

        
        if (gAllWheelErrorFlag==1)
        {
            gstTpmsManager.abnw = 0xf;
            gAllWheelErrorFlag = 0;
        }

        /*对于30个点计算的，需要连续两次报异常；240个点的话一次就够了*/
        if (abnw != 0)
        {
            gstTpmsManager.abnw = abnw;
        }

        if (gstTpmsManager.abnw != 0)
        {
            gstTpmsManager.Tpms_SetAbnw(gstTpmsManager.abnw);
            gstTpmsManager.Tpms_ReportAbnw(gstTpmsManager.abnw);
            
            gstTpmsManager.Tpms_Print("LibTpms: tpms error: %x\r\n", gstTpmsManager.abnw);
            
        }
    }
}



/******************************************************************************
	描    述:   胎压模块初始化
	输入参数:   pstTpms:
	返 回 值:   无
	注意事项:   最先调用
******************************************************************************/
void tpms_Init(TPMS_MANAGER_S *pstTpms)
{
    memcpy(&gstTpmsManager, pstTpms, sizeof(TPMS_MANAGER_S));

    #if USE_ACCE
    tpms_AcceInit();

    #endif
}

/******************************************************************************
	描    述:   胎压工作逻辑
	输入参数:   speed:车速加上轮速
	返 回 值:   无
	注意事项:   无
******************************************************************************/
void tpms_Logic(u16 *speed)
{
    if (gstTpmsManager.learnFlag==0)
    {
        tpms_Calibrate(speed);
    }
    else
    {
        if (gstTpmsManager.abnw == 0)
        {
            tpms_Algorithm(speed);
        }
        
    }
}

/******************************************************************************
	描    述:   获取当前胎压运算的数据
	输入参数:   data:获取的数据(原浮点数值乘以1000000)
	返 回 值:   成功0，失败1
	注意事项:   运算点数不够时返回1
******************************************************************************/
u8 tpms_GetCurData(u32 *data)
{
    u8 i;

    if (gpstTpmsData.dataCount < TPMS_DATA_COUNT-1)
    {
        return 1;
    }
    
    for (i=0; i<4; i++)
    {
        data[i] = (u32)(rtData[i]*1000000);
    }

    return 0;
}

/******************************************************************************
	描    述:   胎压异常复位
	输入参数:   无
	返 回 值:   无
	注意事项:   无
******************************************************************************/
void tpms_ClearError(void)
{
    gstTpmsManager.abnw = 0;
    gstTpmsManager.Tpms_SetAbnw(0);
    
    gpstTpmsData.dataCount = 0;
    gpstTpmsData.dataPointer = 0;
}


/******************************************************************************
	描    述:   胎压重新激活
	输入参数:   无
	返 回 值:   无
	注意事项:   无
******************************************************************************/
void tpms_Learn(void)
{    
    memset(&gpstTpmsData, 0, sizeof(INDIRECT_TPMS_DATA_S));
    
    gstTpmsManager.abnw = 0;
    gstTpmsManager.learnFlag = 0;
    gstTpmsManager.Tpms_SetAbnw(0);
    gstTpmsManager.Tpms_SetLearnFlag(0, NULL);
}

/******************************************************************************
	描    述:   设置4个轮胎异常
	输入参数:   无
	返 回 值:   无
	注意事项:   该函数用于国标测试时人工设置用
******************************************************************************/
void tpms_SetAllWheelError(void)
{
    gAllWheelErrorFlag = 1;
}


/******************************************************************************
	描    述:   获取胎压库的版本号
	输入参数:   无
	返 回 值:   版本号
	注意事项:   无
******************************************************************************/
u32 tpms_GetVersion(void)
{
    gstTpmsManager.Tpms_Print("LibTpms: version %d\r\n", VERSION_TPMS);

    return VERSION_TPMS;
}


