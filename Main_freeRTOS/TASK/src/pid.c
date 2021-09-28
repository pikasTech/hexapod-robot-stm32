/*P �����ٶȼӿ죨���������񵴴������ࣩ
I ������̬ ��߾���
D  ʹ��������С �����ͺ�*/
#include "pid.h"
void PID_Calculate(PID* pid) {
    float DelEk;
    //����Ϊ�򻯱��������ս����������ΪIout
    float Iout, Pout, Dout;
    pid->Ek = pid->Presentvale - pid->Setvalue;  //��ǰƫ��
    Pout = pid->Kp * pid->Ek;                    //���������
                               //������ļ򻯺����
    pid->SumEk += pid->Ek;  //��ʷƫ��֮��
    if (pid->SumEk > 2000.0f)
        pid->SumEk = 2000.0f;
    else if (pid->SumEk < -2000.0f)
        pid->SumEk = -2000.0f;
    else
        ;
    DelEk = pid->Ek - pid->Ek_1;  //�������ƫ��

    Iout = pid->Ki * pid->SumEk;  //���������
    //΢����ļ򻯺����

    Dout = pid->Kd * DelEk;  //΢�������
    //����Ӧ���޷����    ������޷�

    pid->Ek_1 = pid->Ek;                         //����ƫ��
    pid->OUT = Pout + Iout + Dout + pid->OUT_0;  //�ڵ������
}
