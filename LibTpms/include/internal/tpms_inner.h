#ifndef TPMS_INNER_H_
#define TPMS_INNER_H_

#include "lw_type.h"

#define TPMS_DATA_COUNT     240
#define TPMS_THRESHOLD      0.0022


#define USE_ACCE        0       /*使用加速度计来判断转弯和上下坡*/


typedef struct tagIndirectTpmsData
{
    u16 dataPointer;    
    u16 dataCount;      
    u16 tpmsData[5][TPMS_DATA_COUNT];    /**/
    
}INDIRECT_TPMS_DATA_S;


#endif
