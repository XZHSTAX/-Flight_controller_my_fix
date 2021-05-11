#ifndef _OPTICALFLOW_H_
#define _OPTICALFLOW_H_

#include "common.h"

#define Motion_Burst 0x16

#define PMW_RESOLUTION				(0.2131946f)	/*1m高度下 1个像素对应的位移，单位cm*/
#define DEG_TO_RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD_TO_DEG		57.29578f		/* 弧度转度 180/π */

typedef struct
{
	uint8_t motion;
	uint8_t observation;
	
	int16_t deltaX;
	int16_t deltaY;
	
	uint8_t squal;
	uint8_t rawDataSum;
	uint8_t maxRawData;
	uint8_t minRawData;

	uint16_t shutter;
}motionBurst_t;
extern motionBurst_t motionBurst;

typedef struct{
  float SumX;       /*累积像素*/
  float SumY;
  float LpfX;       /*累积像素低通*/
  float LpfY;
  float CompX;      /*倾角补偿*/
  float CompY;
  float DataOutX;   /*补偿像素*/
  float DataOutY;
  float DataDx;     /*2帧之间位移变化量，单位像素*/
  float DataDy;
  float Vx;         /*速度，单位像素/s*/
  float Vy;
  float VxFix;      /*像素低通*/
  float VyFix;
}_PMW3901_pixel_flow;
extern _PMW3901_pixel_flow pmw_pixel_flow;

extern int16_t DY_PMW_OF_DX2, DY_PMW_OF_DY2;
extern int16_t DY_PMW_OF_DX2FIX, DY_PMW_OF_DY2FIX;

extern u8 OpticalFlow_RawData[12];

u8 OpticalFlow_Init(void);
void PMW3901_Read(void);
void PMW3901_Data_Prepare(void);
void OpticalFlow_DataFusion_Task(void);

#endif
