#include "action_task_723.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "rtp_test_task.h"
#include "adc_task.h"
#include "sys.h"
#include "balance_task.h"
#include "pid.h"
#include "touch_task.h"
/*该文件为10.20晚21点之前的副本，仅声明了外部PID变量以及两个新的腿高设置函数，action()函数中未进行修改*/
/*声明外部PID变量来修改固定的摆动态下的3 和XX态下的5*/
extern PID  RF_DIS;
extern PID  RM_DIS;
extern PID  RB_DIS;
extern PID  LF_DIS;
extern PID  LM_DIS;
extern PID  LB_DIS;

extern struct TOUCH touch_use;

/*声明部分LRL 和RLR腿高分别设置*/
void LRL_high_single(int time,float h1,float h2,float h3); //LRL顺序为LF RM LB
void RLR_high_single(int time,float h1,float h2,float h3); //RLR顺序为RF LM RB

/*结束10.20日定义*/
int action_t;
int action_n=0;
float angle_test[6];

#define S10 0
#define S0 	1
#define S01 2
#define S1 	3

//伸展系数
double L_init=7,L_init_best;

//机身高度
double H_init=-13,H_init_best;



//旋转后末端初末位置
double RF_P1z[3];
double RM_P1z[3];
double RB_P1z[3];
double LF_P1z[3];
double LM_P1z[3];
double LB_P1z[3];


//时间起点
long int LRL_t0;
long int RLR_t0;

//过渡占空比
float R0=0.8f;

//过渡提前占空比

float R0f=0.5f;

//前申比例
float Rf=0.5f;

//前转比例
float Rz=0.5f;

//前申比例
float Rf_use=0.5f;

//前转比例
float Rz_use=0.5f;


int STEP_number;//走过的步数

//时间
long int LRL_t;
long int RLR_t;

//相
int LRL_S;
int RLR_S;
//相位
float LRL_phase;
float RLR_phase;

float speed=3;

//周期
float Action_T=2000;

float Action_T_per_V=8400;
//float Action_T_per_V=20000;
//过度相10临界
float phase10;
float RLR_phase10;
//摆动相0临界
float phase0;
float RLR_phase0;
//过度相01临界
float LRL_phase01=0.5f;
float RLR_phase01=0.5f;

//步长
float LRL_step;
float RLR_step;
//移动命令向量 [Vx,Vy,w]
float Order[3]; 
float Order_use[3]; 
//步长向量 [x,y]
float LRL_L[2];
float RLR_L[2];
//旋转角步长
float LRL_theat;
float RLR_theat;
//平移轨迹起点向量 [x1,y1]
float LRL_P1[2];
float RLR_P1[2];
//平移轨迹终点向量 [x2,y2]
float LRL_P2[2];
float RLR_P2[2];
//旋转轨迹起点角度 [theat1]
float LRL_theat1;
float RLR_theat1;
//旋转轨迹终点角度 [theat2]
float LRL_theat2;
float RLR_theat2;

u8 STA_START;//启动标志
u8 STA_STAND=1;//静立标志
u8 STA_STOP=1;//停止标志
u8 STA_TIME_STOP=0;//停时

//向量沿Z轴旋转（旋转矩阵法）
void rotate_z(double input[3],double output[3],double theta)
{
	
	double matrix2[3][1];
	double matrix[3][1];
	double matrix1[3][3];
    int i,j,k;   

	
	matrix2[0][0]=input[0];
  matrix2[1][0]=input[1];
  matrix2[2][0]=input[2];
	
	matrix1[0][0]=cos(theta);
	matrix1[0][1]=-sin(theta);
	matrix1[0][2]=0;
	matrix1[1][0]=sin(theta);
	matrix1[1][1]=cos(theta);
	matrix1[1][2]=0;
	matrix1[2][0]=0;
	matrix1[2][1]=0;
	matrix1[2][2]=1;

    /*???matrix:*/
    for(i=0;i<3;i++){
        for(j=0;j<1;j++){
            matrix[i][j]=0; 
        } 
    } 

    for(i=0;i<3;i++){
        for(j=0;j<1;j++){
            for(k=0;k<3;k++){
                matrix[i][j]=matrix[i][j]+matrix1[i][k]*matrix2[k][j]; 
            } 
        } 
    }

	output[0]=matrix[0][0];
	output[1]=matrix[1][0];
	output[2]=matrix[2][0];		
		
		
}

//机身高度、伸展程度控制
void LH_to_xyz_init(double T03[3][1],double d[4],double L,double H,double xyz_init[3])
{
	
	double t1,t2,t3,t4,t5;
	double c4,s4;
	double d3,d4,d5,d6;
  double x03,y03,z03;
	double x,y,z;
	
	d3=d[0];
	d4=d[1];
	d5=d[2];
	d6=d[3];
	
	x03=T03[0][0];
	y03=T03[1][0];
	z03=T03[2][0];
	//计算 T1 T2
	t1=atan2(y03,x03);
	t2=asin(z03/d3);
	//计算 T3
	t3=0;//初始位置为0		
	//计算 T5
	t5=-acos((L*L+H*H-d5*d5-d6*d6)/(d5*d6*2.0f));
	//计算 T4
	c4=	\

	(d5*L + d6*L*cos(t5) + d6*H*sin(t5))																										\
	/(d6*d6+ d5*d5 + 2.0f*d5*d6*cos(t5));

	s4=	\
	
	(d5*H - d6*L*sin(t5) + d6*H*cos(t5))																										\
	/(d6*d6	+ d5*d5 + 2.0f*d5*d6*cos(t5));
	t4=atan2(s4,c4);
	
	x =		\
	 
	d3*cos(t1)*cos(t2) - d6*(cos(t5)*(cos(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t2)*cos(t3))				\
	+ cos(t1)*sin(t2)*sin(t4)) - sin(t5)*(sin(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t2)*cos(t3))		\
	- cos(t1)*cos(t4)*sin(t2))) - d5*(cos(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t2)*cos(t3))				\
	+ cos(t1)*sin(t2)*sin(t4)) - d4*(sin(t1)*sin(t3) - cos(t1)*cos(t2)*cos(t3))									;
	 
	y =		\
	 
	d4*(cos(t1)*sin(t3) + cos(t2)*cos(t3)*sin(t1)) + d6*(cos(t5)*(cos(t4)*(cos(t1)*sin(t3)			\
	+ cos(t2)*cos(t3)*sin(t1)) - sin(t1)*sin(t2)*sin(t4)) - sin(t5)*(sin(t4)*(cos(t1)*sin(t3)		\
	+ cos(t2)*cos(t3)*sin(t1)) + cos(t4)*sin(t1)*sin(t2))) + d5*(cos(t4)*(cos(t1)*sin(t3)				\
	+ cos(t2)*cos(t3)*sin(t1)) - sin(t1)*sin(t2)*sin(t4)) + d3*cos(t2)*sin(t1)									;
	 
	
	z =		\
	 
	d5*(cos(t2)*sin(t4) + cos(t3)*cos(t4)*sin(t2)) + d6*(cos(t5)*(cos(t2)*sin(t4)								\
	+ cos(t3)*cos(t4)*sin(t2)) + sin(t5)*(cos(t2)*cos(t4) - cos(t3)*sin(t2)*sin(t4)))						\
	+ d3*sin(t2) + d4*cos(t3)*sin(t2)																														;

	
	xyz_init[0]=x;
	xyz_init[1]=y;
	xyz_init[2]=z;
	
}


void position_single(double input[3],float x,float y ,float theta,float h)
{
	 rotate_z(input,LF_P1z,theta);//旋转变换
	 T06_LF[0]=LF_P1z[0]+x;
	 T06_LF[1]=LF_P1z[1]+y;
	 T06_LF[2]=LF_P1z[2]+h;
}

void position_group(struct GROUP_PRA *group_pra)
{
		int i;
		double *input;
		for(i = 1;i < 3;i ++)
		{
		 switch((*group_pra).group)
		 {
			case LRL:
				switch(i)
				{
					case 0 : input = T06_init_LF;
					break;
					case 1 : input = T06_init_RM;
					break;
					case 2 : input = T06_init_LB;
					break;
				}
			break;
			
			case RLR:
				switch(i)
				{
					case 0 : input = T06_init_RF;
					break;
					case 1 : input = T06_init_LM;
					break;
					case 2 : input = T06_init_RB;
					break;
				}
			break;
			}
			position_single(input,(*group_pra).x,(*group_pra).y,(*group_pra).theta,(*group_pra).h);	
		}	
}


float high_from_action_task;
float x_from_action_task;
float y_from_action_task;

void wether_Clear_Order(void)
{
			int i;
			if(STA_STOP==1)for(i=0;i<3;i++)Order_use[i]=0;
			else for(i=0;i<3;i++)Order_use[i]=Order[i];

			for(i=0;i<3;i++)Order_use[i]=Order_use[i];
}
				
void generate_Step_Vector(void)
{
				Action_T=Action_T_per_V/speed;
				//计算平移步长向量
				LRL_L[0]=Order_use[0]*Action_T/1000.0f;
				LRL_L[1]=Order_use[1]*Action_T/1000.0f;

				RLR_L[0]=Order_use[0]*Action_T/1000.0f;
				RLR_L[1]=Order_use[1]*Action_T/1000.0f;

				//计算旋转角步长
				
				LRL_theat=Order_use[2]*Action_T/1000.0f;
	
				RLR_theat=Order_use[2]*Action_T/1000.0f;				
}
				
void generate_Initial_And_End_Position(void)	
{
				if(STA_START==1)Rf_use=0,Rz_use=0;//启动时前伸比和前转比置1
				else Rf_use=Rf,Rz_use=Rz;
				
				//计算平移变换初末位置
				LRL_P1[0]=LRL_L[0]*Rf_use;
				LRL_P1[1]=LRL_L[1]*Rf_use;

				LRL_P2[0]=-LRL_L[0]*(1-Rf_use);
				LRL_P2[1]=-LRL_L[1]*(1-Rf_use);
				
				RLR_P1[0]=RLR_L[0]*Rf_use;
				RLR_P1[1]=RLR_L[1]*Rf_use;

				RLR_P2[0]=-RLR_L[0]*(1-Rf_use);
				RLR_P2[1]=-RLR_L[1]*(1-Rf_use);

				//计算旋转变换初末位置
				LRL_theat1=LRL_theat*Rz_use;
				LRL_theat2=-LRL_theat*(1-Rz_use);
				
				RLR_theat1=RLR_theat*Rz_use;
				RLR_theat2=-RLR_theat*(1-Rz_use);
}				

void wether_Go_To_Next_Cycle()
			{
								//进入下一周期
				if(STA_STAND==0&&LRL_phase>=1)
				{
						LRL_phase=0,LRL_t0=systime,STA_START=0;//结束启动周期，进入正常运行周期
						STEP_number++;
				}
				//进入下一周期
				if(STA_STAND==0&&RLR_phase>=1)
				{
					RLR_phase-=1;
				}
			}
	
void get_phase(void)
{
					//获取当前时间
				LRL_t=systime-LRL_t0;
				printf("C %ld\r\n",systime);
				//获取当前相位
				LRL_phase=LRL_t/Action_T;
				RLR_phase=LRL_phase+0.5f;
				wether_Go_To_Next_Cycle();//判断是否进入下一周期
				printf("D %f\r\n",LRL_phase);				
				printf("E %f\r\n",RLR_phase);

}

void get_order_and_isSTART(void)
{
				if(	(LRL_phase>=0.5f&&LRL_phase<=0.53f)			\
					||(RLR_phase>=0.5f&&RLR_phase<=0.53f)	)
				{
					//判断是否需要切换至静立状态
					if(	(Order_use[0]>-0.2f&&Order_use[0]<0.2f)			\
						&&(Order_use[1]>-0.2f&&Order_use[1]<0.2f)			\
						&&(Order_use[2]>-0.01f&&Order_use[2]<0.01f)			\
						&&(Order_use[0]>-0.2f&&Order_use[0]<0.2f)			\
						&&(Order_use[1]>-0.2f&&Order_use[1]<0.2f)			\
						&&(Order_use[2]>-0.01f&&Order_use[2]<0.01f)			\
						)STA_STAND=1;//切换至静立状态					
				}
				
				if(STA_STAND==1)//静立时检测是否需要启动
				{
					LRL_t0=systime;//静立;
						if(	(Order_use[0]>-0.2f&&Order_use[0]<0.2f)			\
							&&(Order_use[1]>-0.2f&&Order_use[1]<0.2f)			\
							&&(Order_use[2]>-0.01f&&Order_use[2]<0.01f)			\
							&&(Order_use[0]>-0.2f&&Order_use[0]<0.2f)			\
							&&(Order_use[1]>-0.2f&&Order_use[1]<0.2f)			\
							&&(Order_use[2]>-0.01f&&Order_use[2]<0.01f)			\
						);
					else STA_STAND=0,STA_START=1;//需要移动时启动
				}
				
								wether_Clear_Order();	//判断是否停止
}					
float get_S(float phase)
{
				float S_fun;
				//计算相位临界值
				phase10=R0/4.0f;
				phase0=0.5f-phase10;

//				RLR_phase10=R0/4.0f;
//				RLR_phase0=0.5f-phase10;	
				//相切换
				if(phase>=0&&phase<phase10)S_fun=S10;
				if(phase>=phase10&&phase<phase0)S_fun=S0;
				if(phase>=phase0&&phase<LRL_phase01)S_fun=S01;
				if(phase>=LRL_phase01&&phase<1)S_fun=S1;			
				return S_fun;
}

float ZhiXianGuiJi(float x_Start, float x_End ,float y_Start,float y_End,float x)
{
	float k;
	float y;
	k=(y_End-y_Start)/(x_End-x_Start);
	y = y_Start+k*(x-x_Start);
	return y;
}

struct GROUP_PRA LRL_group,RLR_group;

float h_up=5;
float LRL_hNow;
float RLR_hNow;
float h_EWai = 5;	

float XianFu_min(float input,float min)
{
	float output;
	if(input<=min) output = min;
	else output = input;
	return output;
}

void action_init(void)
{
	LRL_group.group = LRL;
	RLR_group.group = RLR;
}

void get_group_pra(struct GROUP_PRA * group_pra , float phase ,u8 STA_STAND_fun, u8 S_fun)
{
	if(STA_STAND_fun==1)
	{
		(*group_pra).x = 0;
		(*group_pra).y = 0;
		(*group_pra).theta = 0;
		(*group_pra).h = 0;
	}

	if(STA_STAND_fun==0)
	{
		switch(S_fun)
		{
			case S01:
				(*group_pra).h = ZhiXianGuiJi(phase0,phase0 + phase10*(1-R0f) ,h_up,0,phase);
			break;
			case S0:
				(*group_pra).x = ZhiXianGuiJi(phase10, phase10 + 0.5f-2*phase10 ,LRL_P2[0],LRL_P1[0],phase);
				(*group_pra).y = ZhiXianGuiJi(phase10, phase10 + 0.5f-2*phase10 ,LRL_P2[1],LRL_P1[1],phase);
				(*group_pra).theta = ZhiXianGuiJi(phase10, phase10 + 0.5f-2*phase10 ,LRL_theat2,LRL_theat1,phase);		
			break;
			case S10:
				(*group_pra).h = ZhiXianGuiJi(phase10*R0f, phase10*R0f + phase10*(1-R0f),0, h_up,phase);
			break;
			case S1:
				(*group_pra).x = ZhiXianGuiJi(0.5f, 0.5f + 0.5f,LRL_P1[0],LRL_P2[0],phase);
				(*group_pra).y = ZhiXianGuiJi(0.5f, 0.5f + 0.5f,LRL_P1[1],LRL_P2[1],phase);
				(*group_pra).theta = ZhiXianGuiJi(0.5f, 0.5f + 0.5f,LRL_theat1,LRL_theat2,phase);
			break;
		}
//		(*group_pra).h = XianFu_min((*group_pra).h,0);

	}
}
void get_position(void)
{

}
#define NORMAL 0
#define TEST 1

u8 STA_TEST_TaiJiao = 1;
u8 STA_ACTION = NORMAL;
float h_TEST = 5;
float time_TEST_TaiJiao = 300;
float time_TEST_LuoJiao = 1000;
float ChaBu_NUM_TEST = 20;
float ChaBu_n_TEST = 0;
float ChaBu_dt;
float ChaBu_hNow;
float ChaBu_hChuDi;

//action任务函数
void action_task(void *pvParameters)
{
	int i;
	struct GROUP_PRA *group_pra;
	float phase;
	float S_fun;
	while(1)
	{
		if(flag_action_run_single==1||flag_action_run_continiue==1)//出现（单次/连续）运行触发信号
		{
				T06_init();//初始化末端位置

				get_order_and_isSTART();//获取指令			

				generate_Step_Vector();//生成平面步长向量
				
				generate_Initial_And_End_Position();//生成平面初末位置

				get_phase();//获得当前相位

				for (i = 0;i<2;i++)//调用两组腿的运动参数，计算两组腿的末端位置
				{
					switch(i)
					{
						case 0 : 
							group_pra = &LRL_group;
							phase = LRL_phase;
							LRL_S = get_S(phase);//获得当前状态	
							printf("F %d\r\n",LRL_S);
							S_fun = LRL_S;
						break;
						case 1 : 
							group_pra = &RLR_group;
							phase = RLR_phase;				
							RLR_S = get_S(phase);//获得当前状态	
							printf("G %d\r\n",RLR_S);
							S_fun = RLR_S;
						break;
					}
					get_group_pra(group_pra,phase,STA_STAND,S_fun);	 //调用一组腿的运动参数
					position_group(group_pra); //计算一组腿的末端位置
				}
				printf("H %f\r\n",LRL_group.h);
				printf("I %f\r\n",LRL_group.x);
				printf("J %f\r\n",LRL_group.y);
				printf("K %f\r\n",LRL_group.theta);

				printf("L %f\r\n",RLR_group.h);
				printf("M %f\r\n",RLR_group.x);
				printf("N %f\r\n",RLR_group.y);
				printf("O %f\r\n",RLR_group.theta);

				
				flag_action_run_single=2;	
				
				vTaskDelay(1);

		}
			vTaskDelay(10);
	}
}
















