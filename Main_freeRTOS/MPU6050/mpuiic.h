#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "stm32f4xx.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))  



#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入 


   	   		   
//IO方向设置
#define MPU_SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}	//PB9输入模式
#define MPU_SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;} //PB9输出模式
//IO操作函数	 
#define MPU_IIC_SCL    PBout(10) //SCL
#define MPU_IIC_SDA    PBout(11) //SDA	 
#define MPU_READ_SDA   PBin(11)  //输入SDA 

//MPU_IIC所有操作函数
void MPU_IIC_Init(void);                //初始化MPU_IIC的IO口				 
int MPU_IIC_Start(void);				//发送MPU_IIC开始信号
void MPU_IIC_Stop(void);	  			//发送MPU_IIC停止信号
void MPU_IIC_Send_Byte(u8 txd);			//MPU_IIC发送一个字节
u8 MPU_IIC_Read_Byte(unsigned char ack);//MPU_IIC读取一个字节
int MPU_IIC_Wait_Ack(void); 				//MPU_IIC等待ACK信号
void MPU_IIC_Ack(void);					//MPU_IIC发送ACK信号
void MPU_IIC_NAck(void);				//MPU_IIC不发送ACK信号

void MPU_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	 
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char MPU_IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 MPU_IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 MPU_IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 MPU_IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 MPU_IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif

//------------------End of File----------------------------
