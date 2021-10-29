#include "mpuIIC.h"
#include "sys.h"
#include "delay.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
/**************************实现函数********************************************
*函数原型:      void MPU_IIC_Init(void)
*功　　能:      初始化I2C对应的接口引脚。
*******************************************************************************/
void MPU_IIC_Init(void)
{
GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

  //GPIOB8,B9初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
    MPU_IIC_SCL=1;
    MPU_IIC_SDA=1;
}

/**************************实现函数********************************************
*函数原型:      void MPU_IIC_Start(void)
*功　　能:      产生MPU_IIC起始信号
*******************************************************************************/
int MPU_IIC_Start(void)
{
    MPU_SDA_OUT();     //sda线输出
    MPU_IIC_SDA=1;
    if(!MPU_READ_SDA)return 0;
    MPU_IIC_SCL=1;
    delay_us(1);
    MPU_IIC_SDA=0;//START:when CLK is high,DATA change form high to low
    if(MPU_READ_SDA)return 0;
    delay_us(1);
    MPU_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
    return 1;
}

/**************************实现函数********************************************
*函数原型:      void MPU_IIC_Stop(void)
*功　　能:      //产生MPU_IIC停止信号
*******************************************************************************/
void MPU_IIC_Stop(void)
{
    MPU_SDA_OUT();//sda线输出
    MPU_IIC_SCL=0;
    MPU_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
    delay_us(1);
    MPU_IIC_SCL=1;
    MPU_IIC_SDA=1;//发送I2C总线结束信号
    delay_us(1);
}

/**************************实现函数********************************************
*函数原型:      u8 MPU_IIC_Wait_Ack(void)
*功　　能:      等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
int MPU_IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    MPU_SDA_IN();      //SDA设置为输入
    MPU_IIC_SDA=1;
    delay_us(1);
    MPU_IIC_SCL=1;
    delay_us(1);
    while(MPU_READ_SDA)
    {
        ucErrTime++;
        if(ucErrTime>50)
        {
            MPU_IIC_Stop();
            return 0;
        }
      delay_us(1);
    }
    MPU_IIC_SCL=0;//时钟输出0
    return 1;
}

/**************************实现函数********************************************
*函数原型:      void MPU_IIC_Ack(void)
*功　　能:      产生ACK应答
*******************************************************************************/
void MPU_IIC_Ack(void)
{
    MPU_IIC_SCL=0;
    MPU_SDA_OUT();
    MPU_IIC_SDA=0;
    delay_us(1);
    MPU_IIC_SCL=1;
    delay_us(1);
    MPU_IIC_SCL=0;
}

/**************************实现函数********************************************
*函数原型:      void MPU_IIC_NAck(void)
*功　　能:      产生NACK应答
*******************************************************************************/
void MPU_IIC_NAck(void)
{
    MPU_IIC_SCL=0;
    MPU_SDA_OUT();
    MPU_IIC_SDA=1;
    delay_us(1);
    MPU_IIC_SCL=1;
    delay_us(1);
    MPU_IIC_SCL=0;
}
/**************************实现函数********************************************
*函数原型:      void MPU_IIC_Send_Byte(u8 txd)
*功　　能:      MPU_IIC发送一个字节
*******************************************************************************/
void MPU_IIC_Send_Byte(u8 txd)
{
    u8 t;
    MPU_SDA_OUT();
    MPU_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
        MPU_IIC_SDA=(txd&0x80)>>7;
        txd<<=1;
        delay_us(1);
        MPU_IIC_SCL=1;
        delay_us(1);
        MPU_IIC_SCL=0;
        delay_us(1);
    }
}

/**************************实现函数********************************************
*函数原型:      bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*功　　能:
*******************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
        int i;
    if (!MPU_IIC_Start())
        return 1;
    MPU_IIC_Send_Byte(addr << 1 );
    if (!MPU_IIC_Wait_Ack()) {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
        for (i = 0; i < len; i++) {
        MPU_IIC_Send_Byte(data[i]);
        if (!MPU_IIC_Wait_Ack()) {
            MPU_IIC_Stop();
            return 0;
        }
    }
    MPU_IIC_Stop();
    return 0;
}
/**************************实现函数********************************************
*函数原型:      bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*功　　能:
*******************************************************************************/
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!MPU_IIC_Start())
        return 1;
    MPU_IIC_Send_Byte(addr << 1);
    if (!MPU_IIC_Wait_Ack()) {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1)+1);
    MPU_IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *buf = MPU_IIC_Read_Byte(0);
        else
            *buf = MPU_IIC_Read_Byte(1);
        buf++;
        len--;
    }
    MPU_IIC_Stop();
    return 0;
}


/**************************实现函数********************************************
*函数原型:      u8 MPU_IIC_Read_Byte(unsigned char ack)
*功　　能:      //读1个字节，ack=1时，发送ACK，ack=0，发送nACK
*******************************************************************************/
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    MPU_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
    {
        MPU_IIC_SCL=0;
        delay_us(2);
        MPU_IIC_SCL=1;
        receive<<=1;
        if(MPU_READ_SDA)receive++;
        delay_us(2);
    }
    if (ack)
        MPU_IIC_Ack(); //发送ACK
    else
        MPU_IIC_NAck();//发送nACK
    return receive;
}

/**************************实现函数********************************************
*函数原型:      unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:      读取指定设备 指定寄存器的一个值
输入  I2C_Addr  目标设备地址
        addr       寄存器地址
返回   读出来的值
*******************************************************************************/
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
    unsigned char res=0;

    MPU_IIC_Start();
    MPU_IIC_Send_Byte(I2C_Addr);       //发送写命令
    res++;
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(addr); res++;  //发送地址
    MPU_IIC_Wait_Ack();
    //MPU_IIC_Stop();//产生一个停止条件
    MPU_IIC_Start();
    MPU_IIC_Send_Byte(I2C_Addr+1); res++;          //进入接收模式
    MPU_IIC_Wait_Ack();
    res=MPU_IIC_Read_Byte(0);
    MPU_IIC_Stop();//产生一个停止条件

    return res;
}


/**************************实现函数********************************************
*函数原型:      u8 MPU_IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:      读取指定设备 指定寄存器的 length个值
输入  dev  目标设备地址
        reg   寄存器地址
        length 要读的字节数
        *data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/
u8 MPU_IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;

    MPU_IIC_Start();
    MPU_IIC_Send_Byte(dev);    //发送写命令
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(reg);   //发送地址
    MPU_IIC_Wait_Ack();
    MPU_IIC_Start();
    MPU_IIC_Send_Byte(dev+1);  //进入接收模式
    MPU_IIC_Wait_Ack();

    for(count=0;count<length;count++){

         if(count!=length-1)data[count]=MPU_IIC_Read_Byte(1);  //带ACK的读数据
            else  data[count]=MPU_IIC_Read_Byte(0);  //最后一个字节NACK
    }
    MPU_IIC_Stop();//产生一个停止条件
    return count;
}

/**************************实现函数********************************************
*函数原型:      u8 MPU_IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:      将多个字节写入指定设备 指定寄存器
输入  dev  目标设备地址
        reg   寄存器地址
        length 要写的字节数
        *data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/
u8 MPU_IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){

    u8 count = 0;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte(dev);    //发送写命令
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(reg);   //发送地址
    MPU_IIC_Wait_Ack();
    for(count=0;count<length;count++){
        MPU_IIC_Send_Byte(data[count]);
        MPU_IIC_Wait_Ack();
     }
    MPU_IIC_Stop();//产生一个停止条件

    return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:      u8 MPU_IICreadByte(u8 dev, u8 reg, u8 *data)
*功　　能:      读取指定设备 指定寄存器的一个值
输入  dev  目标设备地址
        reg    寄存器地址
        *data  读出的数据将要存放的地址
返回   1
*******************************************************************************/
u8 MPU_IICreadByte(u8 dev, u8 reg, u8 *data){
    *data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************实现函数********************************************
*函数原型:      unsigned char MPU_IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:      写入指定设备 指定寄存器一个字节
输入  dev  目标设备地址
        reg    寄存器地址
        data  将要写入的字节
返回   1
*******************************************************************************/
unsigned char MPU_IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return MPU_IICwriteBytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:      u8 MPU_IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:      读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入  dev  目标设备地址
        reg    寄存器地址
        bitStart  目标字节的起始位
        length   位长度
        data    存放改变目标字节位的值
返回   成功 为1
        失败为0
*******************************************************************************/
u8 MPU_IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (MPU_IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return MPU_IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************实现函数********************************************
*函数原型:      u8 MPU_IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:      读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入  dev  目标设备地址
        reg    寄存器地址
        bitNum  要修改目标字节的bitNum位
        data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1
        失败为0
*******************************************************************************/
u8 MPU_IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    MPU_IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return MPU_IICwriteByte(dev, reg, b);
}

//------------------End of File----------------------------
