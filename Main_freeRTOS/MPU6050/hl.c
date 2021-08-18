#include "hl.h"




void action_angle(u8 number,double angle,int time)//单关节控制(角度制)
{
	int init;
	int angle_out;
	float k=K_angle;
	switch(number)
		{
			case LB1  :
				init=LB1_init;k=K_angle;angle+=18.914;break;
			case LB2  :
				init=LB2_init;k=K_angle;break;
			case LB3  :
				init=LB3_init;k=-K_angle;break;
			case LM1	:
				init=LM1_init;k=K_angle;break;
			case LM2	:
				init=LM2_init;k=K_angle;break;
			case LM3	:
				init=LM3_init;k=-K_angle;break;
			case LF1	:
				init=LF1_init;k=K_angle;angle-=18.914;break;
			case LF2	:
				init=LF2_init;k=K_angle;break;
			case LF3	:
				init=LF3_init;k=-K_angle;break;
			case RF1	:
				init=RF1_init;k=K_angle;angle+=18.914;break;
			case RF2	:
				init=RF2_init;k=-K_angle;break;
			case	RF3:
				init=RF3_init;k=+K_angle;break;
			case	RM1:
				init=RM1_init;k=+K_angle;break;
			case	RM2:
				init=RM2_init;k=-K_angle;break;
			case	RM3:
				init=RM3_init;k=+K_angle;break;
			case	RB1:
				init=RB1_init;k=+K_angle;angle-=18.914;break;
			case	RB2:
				init=RB2_init;k=-K_angle;break;
			case	RB3:
				init=RB3_init;k=+K_angle;break;
			
    }
		
	angle_out=init+(int)(k*angle);
//	////printf("#%d P%d T%d\r\n",number,angle_out,time);
//		moveServo(number, angle_out, time);
		servos_array[number-1].ID=number;
		servos_array[number-1].Position=angle_out;
}




void action_foot(u8 foot,double angle1,double angle2,double angle3,int time)//单腿全变量控制
{
	if(foot==LB)
	{
		action_angle(LB1,angle1,time);
		action_angle(LB2,angle2,time);
		action_angle(LB3,angle3,time);		
	}
	
	else if(foot==LM)
	{
		action_angle(LM1,angle1,time);
		action_angle(LM2,angle2,time);
		action_angle(LM3,angle3,time);		
	}
	
		else if(foot==LF)
	{
		action_angle(LF1,angle1,time);
		action_angle(LF2,angle2,time);
		action_angle(LF3,angle3,time);		
	}
	
		else if(foot==RF)
	{
		action_angle(RF1,angle1,time);
		action_angle(RF2,angle2,time);
		action_angle(RF3,angle3,time);		
	}
	
		else if(foot==RM)
	{
		action_angle(RM1,angle1,time);
		action_angle(RM2,angle2,time);
		action_angle(RM3,angle3,time);		
	}
	
		else if(foot==RB)
	{
		action_angle(RB1,angle1,time);
		action_angle(RB2,angle2,time);
		action_angle(RB3,angle3,time);		
	}
	
}



void action_angle_test(u8 number,int angle_out,u8 time)//单关节调试
{
	////printf("#%d P%d T%d\r\n",number,angle_out,time);
			moveServo(number, angle_out, time);
}

