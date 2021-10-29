#include "control.h"
#include "filter.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "sys.h"
#include "math.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target;
u32 Flash_R_Count;
int Voltage_Temp,Voltage_Count,Voltage_All;
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步
**************************************************************************/
//void TIM3_IRQHandler(void)
//{
//  if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
//  {
//         TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位

//}
//}

void Get_Angle(u8 way)
{
        float Accel_Y,Accel_Angle_X,Accel_Angle_Y,Accel_X,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z;
//      Temperature=Read_Temperature();      //===读取MPU6050内置温度传感器数据，近似表示主板温度。
        if(way==1)                           //===DMP的读取在数据采集中断读取，严格遵循时序要求
            {
                    Read_DMP();                      //===读取加速度、角速度、倾角
                    Angle_Balance=-Roll;             //===更新平衡倾角
                    Gyro_Balance=-gyro[0];            //===更新平衡角速度
                    Gyro_Turn=gyro[2];               //===更新转向角速度
                    Gyro_Pitch=gyro[1]/100.0;
                    Gyro_Roll=gyro[0]/100.0;
//                Acceleration_Z=accel[2];         //===更新Z轴加速度计
            }
      else
      {
            Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取Y轴陀螺仪
            Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
            Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪

          Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
          Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
            Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计

            if(Gyro_X>32768)  Gyro_X-=65536;                       //数据类型转换  也可通过short强制类型转换
            if(Gyro_Y>32768)  Gyro_Y-=65536;                       //数据类型转换
            if(Gyro_Z>32768)  Gyro_Z-=65536;                       //数据类型转换

        if(Accel_X>32768) Accel_X-=65536;                      //数据类型转换
            if(Accel_Y>32768) Accel_Y-=65536;                      //数据类型转换
          if(Accel_Z>32768) Accel_Z-=65536;                      //数据类型转换

            Gyro_Balance=Gyro_X;                                  //更新平衡角速度
        Accel_Angle_X=atan2(Accel_Y,Accel_Z)*180/PI;                 //计算倾角
        Accel_Angle_Y=-atan2(Accel_X,Accel_Z)*180/PI;                 //计算倾角

            Gyro_X=Gyro_X/16.4f;                                    //陀螺仪量程转换
            Gyro_Y=Gyro_Y/16.4f;
      if(Way_Angle==2)
            {
//              Kalman_Filter_x(Accel_Angle_X,Gyro_X);//卡尔曼滤波
                Roll=angle_x;
//              Kalman_Filter_y(Accel_Angle_Y,Gyro_Y);//卡尔曼滤波
                Pitch=angle_y;
            }
            else if(Way_Angle==3)
            {
                Yijielvbo_x(Accel_Angle_X,Gyro_X);    //互补滤波
                Roll=angle_x;
                Yijielvbo_y(Accel_Angle_Y,Gyro_Y);    //互补滤波
                Pitch=angle_y;
            }
        }
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{
      int temp;
        if(a<0)  temp=-a;
      else temp=a;
      return temp;

}

