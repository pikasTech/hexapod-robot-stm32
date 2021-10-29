/*P 调节速度加快（超量增大，振荡次数增多）
I 消除稳态 提高精度
D  使超调量减小 抵消滞后*/
#include "pid.h"
void PID_Calculate(PID *pid)
{   float DelEk;
      //下面为简化变量，最终将积分项反馈化为Iout
      float Iout,Pout,Dout;
      pid->Ek=pid->Presentvale-pid->Setvalue; //当前偏差
      Pout=pid->Kp*pid->Ek;      //比例项输出
        //积分项的简化和输出
      pid->SumEk+=pid->Ek;          //历史偏差之和
      if(pid->SumEk>2000.0f) pid->SumEk=2000.0f;
      else if(pid->SumEk<-2000.0f) pid->SumEk=-2000.0f;
      else ;
      DelEk=pid->Ek-pid->Ek_1;      //最近两次偏差

      Iout=pid->Ki*pid->SumEk;//积分项输出
      //微分项的简化和输出

      Dout=pid->Kd*DelEk;   //微分项输出
      //本次应该限幅输出    电机中限幅

        pid->Ek_1=pid->Ek; //更新偏差
      pid->OUT=Pout+Iout+Dout+pid->OUT_0; //在电机部分
}

