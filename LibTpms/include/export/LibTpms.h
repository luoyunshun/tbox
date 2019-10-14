#ifndef LIB_TPMS_H_
#define LIB_TPMS_H_

#define ABNW_LF     0x08        
#define ABNW_RF     0x04
#define ABNW_LB     0x02
#define ABNW_RB     0x01

typedef struct tagTpmsManager
{
	unsigned char abnw;         /*bit or of ABNW_LF ABNW_RF ABNW_LB ABNW_LB*/
	unsigned char learnFlag;    /*0:not learned 1:learned*/
	float tpmsOffset[4];        /*learn result for TPMS*/
    
	void (*Tpms_SetAbnw)(unsigned char abnw);       /*function for save abnormal wheel pos*/
	void (*Tpms_ReportAbnw)(unsigned char abnw);    /*function for report abnormal wheel pos*/
	void (*Tpms_SetLearnFlag)(unsigned char flag, float *tpmsOffset); /*function for save learn flag, if flag is 0, tpmsOffset should not be used */
	int (*Tpms_Print)(const char *fmt, ...);
}TPMS_MANAGER_S;



/*外部接口*/
/*first called*/
void tpms_Init(TPMS_MANAGER_S *pstTpms);

/*called 1 time per second*/
void tpms_Logic(unsigned short *speed);

/*clear tpms error, called when IC send the signal*/
void tpms_ClearError(void);

/*start tpms learn, called when when IC send the signal*/
void tpms_Learn(void);

/*Get LibTpms version*/
unsigned int tpms_GetVersion(void);

/*set 4 wheels error*/
void tpms_SetAllWheelError(void);

/*set speed area for test*/
void tpms_SetSpeedRange(unsigned short min, unsigned short max);

/*get current status*/
unsigned char tpms_GetCurData(unsigned int *data);



#endif
