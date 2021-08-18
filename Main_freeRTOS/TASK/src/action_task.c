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
float LRL_phase10;
float RLR_phase10;
//摆动相0临界
float LRL_phase0;
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
void rotate_z(double input[3],double output[3],double theat)
{
	
	double matrix2[3][1];
	double matrix[3][1];
	double matrix1[3][3];
    int i,j,k;   

	
	matrix2[0][0]=input[0];
  matrix2[1][0]=input[1];
  matrix2[2][0]=input[2];
	
	matrix1[0][0]=cos(theat);
	matrix1[0][1]=-sin(theat);
	matrix1[0][2]=0;
	matrix1[1][0]=sin(theat);
	matrix1[1][1]=cos(theat);
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

//LRL组 腿高设置
void LRL_high(int time,float h)
{
	 if(LRL_S ==S01)
	 {
		 if(STA_ChuDi_lf == 0)T06_LF[2]=T06_init_LF[2]+h;	
		 if(STA_ChuDi_rm == 0)T06_RM[2]=T06_init_RM[2]+h;
		 if(STA_ChuDi_lb == 0)T06_LB[2]=T06_init_LB[2]+h;	
	 }
	 else
	 {
		 T06_LF[2]=T06_init_LF_ideal[2]+h;	
		 T06_RM[2]=T06_init_RM_ideal[2]+h;
		 T06_LB[2]=T06_init_LB_ideal[2]+h;	
	 }
		 vTaskDelay(time);
}


//LRL组 腿高设置
void LRL_high_ideal(int time,float h)
{
	 if(LRL_S ==S01)
	 {
		 if(STA_ChuDi_lf == 0)T06_LF[2]=T06_init_LF_ideal[2]+h;	
		 if(STA_ChuDi_rm == 0)T06_RM[2]=T06_init_RM_ideal[2]+h;
		 if(STA_ChuDi_lb == 0)T06_LB[2]=T06_init_LB_ideal[2]+h;	
	 }
	 else
	 {
		 T06_LF[2]=T06_init_LF_ideal[2]+h;	
		 T06_RM[2]=T06_init_RM_ideal[2]+h;
		 T06_LB[2]=T06_init_LB_ideal[2]+h;	
	 }
		 vTaskDelay(time);
}

////LRL组 位置设置
void LRL_position(int time,float x,float y,float theat)
{
	 rotate_z(T06_init_LF,LF_P1z,theat);//旋转变换
	 T06_LF[0]=LF_P1z[0]+x;
	 T06_LF[1]=LF_P1z[1]+y;

	 rotate_z(T06_init_RM,RM_P1z,theat);//旋转变换
	 T06_RM[0]=RM_P1z[0]+x;
	 T06_RM[1]=RM_P1z[1]+y;

	 rotate_z(T06_init_LB,LB_P1z,theat);//旋转变换
	 T06_LB[0]=LB_P1z[0]+x;
	 T06_LB[1]=LB_P1z[1]+y;	
	
	
		 vTaskDelay(time);
	
}



//RLR组 腿高设置
void RLR_high(int time,float h )
{
		 if(RLR_S ==S01)
	 {
		 if(STA_ChuDi_rf == 0)T06_RF[2]=T06_init_RF[2]+h;	
		 if(STA_ChuDi_lm == 0)T06_LM[2]=T06_init_LM[2]+h;
		 if(STA_ChuDi_rb == 0)T06_RB[2]=T06_init_RB[2]+h;	
	 }
	 else
	 {
		 T06_RF[2]=T06_init_RF_ideal[2]+h;	
		 T06_LM[2]=T06_init_LM_ideal[2]+h;
		 T06_RB[2]=T06_init_RB_ideal[2]+h;	
	 }
		vTaskDelay(time);
	
}
void RLR_high_single(int time, float h1,float h2,float h3)
{
	 T06_RF[2]=T06_init_RF[2]+h1;	
	 T06_LM[2]=T06_init_LM[2]+h2;
   T06_RB[2]=T06_init_RB[2]+h3;	
		 vTaskDelay(time);
}

////RLR组 位置设置
void RLR_position(int time,float x,float y,float theat)
{
	 rotate_z(T06_init_RF,RF_P1z,theat);//旋转变换
	 T06_RF[0]=RF_P1z[0]+x;
	 T06_RF[1]=RF_P1z[1]+y;

	 rotate_z(T06_init_LM,LM_P1z,theat);//旋转变换
	 T06_LM[0]=LM_P1z[0]+x;	
	 T06_LM[1]=LM_P1z[1]+y;	
	
	 rotate_z(T06_init_RB,RB_P1z,theat);//旋转变换
	 T06_RB[0]=RB_P1z[0]+x;		
	 T06_RB[1]=RB_P1z[1]+y;	
	
	 vTaskDelay(time);
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
void gain_Phase(void)
{
					//获取当前时间
				LRL_t=systime-LRL_t0;
				
				//获取当前相位
				LRL_phase=LRL_t/Action_T;
				RLR_phase=LRL_phase+0.5f;
					
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
void startUp_Or_End(void)
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
void wether_Change_State(void)
{
								//计算相位临界值
				LRL_phase10=R0/4.0f;
				LRL_phase0=0.5f-LRL_phase10;


//				RLR_phase10=R0/4.0f;
//				RLR_phase0=0.5f-LRL_phase10;	
				//相切换
				if(LRL_phase>=0&&LRL_phase<LRL_phase10)LRL_S=S10;
				if(LRL_phase>=LRL_phase10&&LRL_phase<LRL_phase0)LRL_S=S0;
				
				if(LRL_S!=S01 && LRL_phase>=LRL_phase0&&LRL_phase<LRL_phase01)
				{
					LRL_S=S01;
					STA_ChuDi_lf = 0;//未触地
					STA_ChuDi_rm = 0;
					STA_ChuDi_lb = 0;	
				}				
				
				if(LRL_S!= S1 && LRL_phase>=LRL_phase01&&LRL_phase<1)
				{
					float LRL_aver;
					LRL_S=S1;			
					LRL_aver = (T06_init_LF[2] + T06_init_RM[2] + T06_init_LB[2])/3.0f;
					T06_init_LF[2] = T06_init_LF[2] + T06_init_LF_ideal[2] - LRL_aver;
					T06_init_RM[2] = T06_init_RM[2] + T06_init_RM_ideal[2] - LRL_aver;
					T06_init_LB[2] = T06_init_LB[2] + T06_init_LB_ideal[2] - LRL_aver;					
				}
				//相切换
				if(RLR_phase>=0&&RLR_phase<LRL_phase10)RLR_S=S10;
				if(RLR_phase>=LRL_phase10&&RLR_phase<LRL_phase0)RLR_S=S0;
				if(RLR_S!=S01 && RLR_phase>=LRL_phase0&&RLR_phase<LRL_phase01)
				{
					RLR_S=S01;
					STA_ChuDi_rf = 0;//未触地
					STA_ChuDi_lm = 0;
					STA_ChuDi_rb = 0;	
				}
				if(RLR_S!=S1 && RLR_phase>=LRL_phase01&&RLR_phase<1)
				{
					float RLR_aver;
					RLR_S=S1;
					RLR_aver = (T06_init_RF[2] + T06_init_LM[2] + T06_init_RB[2])/3.0f;
					T06_init_RF[2] = T06_init_RF[2] + T06_init_RF_ideal[2] - RLR_aver;
					T06_init_LM[2] = T06_init_LM[2] + T06_init_LM_ideal[2] - RLR_aver;
					T06_init_RB[2] = T06_init_RB[2] + T06_init_RB_ideal[2] - RLR_aver;
					
				}
}

float ZhiXianGuiJi(float x_Start, float x_End ,float y_Start,float y_End,float x)
{
	float k;
	float y;
	k=(y_End-y_Start)/(x_End-x_Start);
	y = y_Start+k*(x-x_Start);
	return y;
}


float h_up=5;
float LRL_hNow;
float RLR_hNow;
float h_EWai = 5;	
void action()
{

				//静立状态时保持静立
				if(STA_STAND==1)
				{
					LRL_position(0,0,0,0);
					RLR_position(0,0,0,0);
					LRL_high(0,0);
					RLR_high(0,0);
				}
				
				//运动状态时执行动作
				if(STA_STAND==0)
				{
					//过度相01
					if(LRL_S==S01)
					{
						float kh;
						float h;
					
//						STA_ChuDi_lf = 0;//未触地
//						STA_ChuDi_rm = 0;
//						STA_ChuDi_lb = 0;	


						#if USE_T06_ideal
						
						  h = ZhiXianGuiJi(LRL_phase0,LRL_phase0 + LRL_phase10*(1-R0f) ,h_up,0,LRL_phase);

							if(h<=0-h_EWai)h=0-h_EWai;
							LRL_hNow = h;
//							printf("[%f]",RLR_hNow);
							//未触地时保持下降
							if(STA_ChuDi_lf == 0)
							{	
								T06_LF[2] =T06_init_LF_ideal[2]+h;
							}
//							else T06_LF[2] = T06_init_LF[2];//锁定实际高度
							if(STA_ChuDi_rm == 0)T06_RM[2]=T06_init_RM_ideal[2]+h;
							if(STA_ChuDi_lb == 0)T06_LB[2]=T06_init_LB_ideal[2]+h;	
							
						#else
							h = ZhiXianGuiJi(LRL_phase0,LRL_phase0 + LRL_phase10*(1-R0f) ,h_up,0,LRL_phase);

							if(h<=0)h=0;
							LRL_hNow = h;
//							printf("[%f]",LRL_hNow);
							LRL_high(0,h);//落腿
							
						#endif
					}
					//摆动相0
					else if(LRL_S==S0)
					{
						//根据末端初末位置生成直线轨迹
						float x;
						float y;
						float theat;

						x = ZhiXianGuiJi(LRL_phase10, LRL_phase10 + 0.5f-2*LRL_phase10 ,LRL_P2[0],LRL_P1[0],LRL_phase);
						y = ZhiXianGuiJi(LRL_phase10, LRL_phase10 + 0.5f-2*LRL_phase10 ,LRL_P2[1],LRL_P1[1],LRL_phase);
						theat = ZhiXianGuiJi(LRL_phase10, LRL_phase10 + 0.5f-2*LRL_phase10 ,LRL_theat2,LRL_theat1,LRL_phase);

						LRL_position(0,x,y,theat);
					}
					//过度相10					
					else if(LRL_S==S10)
					{
						#if USE_T06_ideal
						
						float LF_h;
						float LF_h0;
						float LF_hEnd;

						float RM_h;
						float RM_h0;
						float RM_hEnd;
						
						float LB_h;
						float LB_h0;
						float LB_hEnd;						
						
						//LF
						LF_h0 = T06_init_LF[2] - T06_init_LF_ideal[2] ; //起点
						LF_hEnd = h_up; //终点
						
						if(LRL_phase<LRL_phase10*R0f)LF_h=0;
						else	LF_h=ZhiXianGuiJi(LRL_phase10*R0f, LRL_phase10*R0f + LRL_phase10*(1-R0f),LF_h0,LF_hEnd, LRL_phase);
						T06_LF[2]=T06_init_LF_ideal[2]+LF_h;
						
						//RM
						RM_h0 = T06_init_RM[2] - T06_init_RM_ideal[2] ; //起点
						RM_hEnd = h_up; //终点
						
						if(LRL_phase<LRL_phase10*R0f)RM_h=0;
						else	LF_h=ZhiXianGuiJi(LRL_phase10*R0f, LRL_phase10*R0f + LRL_phase10*(1-R0f),RM_h0,RM_hEnd, LRL_phase);
						T06_RM[2]=T06_init_RM_ideal[2]+RM_h;

						//LB
						LB_h0 = T06_init_LB[2] - T06_init_LB_ideal[2] ; //起点
						LB_hEnd = h_up; //终点
						
						if(LRL_phase<LRL_phase10*R0f)LB_h=0;
						else	LF_h=ZhiXianGuiJi(LRL_phase10*R0f, LRL_phase10*R0f + LRL_phase10*(1-R0f),LB_h0,LB_hEnd, LRL_phase);
						T06_LB[2]=T06_init_LB_ideal[2]+LB_h;						
						
						
						#else
						float h;
						if(LRL_phase<LRL_phase10*R0f)h=0;
						else	h=ZhiXianGuiJi(LRL_phase10*R0f, LRL_phase10*R0f + LRL_phase10*(1-R0f),0,h_up, LRL_phase);
						LRL_high(0,h);//抬腿
						#endif
					}
					//支撑相1
					else if(LRL_S==S1)
					{
						//根据末端初末位置生成直线轨迹
						float x;
						float y;
						float theat;

						x = ZhiXianGuiJi(0.5f, 0.5f + 0.5f,LRL_P1[0],LRL_P2[0],LRL_phase);
						y = ZhiXianGuiJi(0.5f, 0.5f + 0.5f,LRL_P1[1],LRL_P2[1],LRL_phase);
						theat = ZhiXianGuiJi(0.5f, 0.5f + 0.5f,LRL_theat1,LRL_theat2,LRL_phase);

						LRL_position(0,x,y,theat);
					}					

					
					
					//过度相01
					if(RLR_S==S01)
					{
						float kh;
						float h;
						
//						STA_ChuDi_rf = 0;//未触地
//						STA_ChuDi_lm = 0;
//						STA_ChuDi_rb = 0;	

						#if USE_T06_ideal
						
							h=ZhiXianGuiJi(LRL_phase0, LRL_phase0 + LRL_phase10*(1-R0f),h_up,0, RLR_phase);
							if(h<=0-h_EWai)h=0-h_EWai;

//							printf("[%f]",LRL_hNow);
							//未触地时保持下降
							if(STA_ChuDi_rf == 0)
							{	
								T06_RF[2] =T06_init_RF_ideal[2]+h;
							}
//							else T06_LF[2] = T06_init_LF[2];//锁定实际高度
							if(STA_ChuDi_lm == 0)T06_LM[2]=T06_init_LM_ideal[2]+h;
							if(STA_ChuDi_rb == 0)T06_RB[2]=T06_init_RB_ideal[2]+h;	
							
						#else

						h=ZhiXianGuiJi(LRL_phase0, LRL_phase0 + LRL_phase10*(1-R0f),h_up,0, RLR_phase);

						LRL_hNow = h;
						if(h<=0)h=0;
						RLR_high(0,h);//落腿	
						#endif
					}
					//摆动相0
					else if(RLR_S==S0)
					{
						//根据末端初末位置生成直线轨迹
						float kx;
						float ky;
						float ktheat;
						kx=(RLR_P1[0]-RLR_P2[0])/(0.5f-2*LRL_phase10);
						ky=(RLR_P1[1]-RLR_P2[1])/(0.5f-2*LRL_phase10);
						ktheat=(RLR_theat1-RLR_theat2)/(0.5f-2*LRL_phase10);
						RLR_position(0,	RLR_P2[0]+kx*(RLR_phase-LRL_phase10),RLR_P2[1]+ky*(RLR_phase-LRL_phase10),RLR_theat2+ktheat*(RLR_phase-LRL_phase10));
					}
					//过度相10					
					else if(RLR_S==S10)
					{
						#if USE_T06_ideal
						
						float RF_kh;
						float RF_h;
						float RF_h0;
						float RF_hEnd;

						float LM_kh;
						float LM_h;
						float LM_h0;
						float LM_hEnd;
						
						float RB_kh;
						float RB_h;
						float RB_h0;
						float RB_hEnd;						
						
						//RF
						RF_h0 = T06_init_RF[2] - T06_init_RF_ideal[2] ; //起点
						RF_hEnd = h_up; //终点
						
						RF_kh=(RF_hEnd-RF_h0)/(LRL_phase10*(1-R0f));
						if(RLR_phase<LRL_phase10*R0f)RF_h=0;
						else	RF_h=RF_h0+RF_kh*(RLR_phase-LRL_phase10*R0f);
						T06_RF[2]=T06_init_RF_ideal[2]+RF_h;
						
						//LM
						LM_h0 = T06_init_LM[2] - T06_init_LM_ideal[2] ; //起点
						LM_hEnd = h_up; //终点
						
						LM_kh=(LM_hEnd-LM_h0)/(LRL_phase10*(1-R0f));
						if(RLR_phase<LRL_phase10*R0f)LM_h=0;
						else	LM_h=LM_h0+LM_kh*(RLR_phase-LRL_phase10*R0f);
						T06_LM[2]=T06_init_LM_ideal[2]+LM_h;

						//RB
						RB_h0 = T06_init_RB[2] - T06_init_RB_ideal[2] ; //起点
						RB_hEnd = h_up; //终点
						
						RB_kh=(RB_hEnd-RB_h0)/(LRL_phase10*(1-R0f));
						if(RLR_phase<LRL_phase10*R0f)RB_h=0;
						else	RB_h=RB_h0+RB_kh*(RLR_phase-LRL_phase10*R0f);
						T06_RB[2]=T06_init_RB_ideal[2]+RB_h;						
						
						
						#else
						float kh;
						float h;
						kh=(h_up-0)/(LRL_phase10*(1-R0f));
						if(RLR_phase<LRL_phase10*R0f)h=0;
						else	h=0+kh*(RLR_phase-LRL_phase10*R0f);
						RLR_high(0,h);//抬腿
//						printf("[%f,%f]",h,RLR_phase);
						#endif
					}
					//支撑相1					
					else if(RLR_S==S1)
					{
						//根据末端初末位置生成直线轨迹
						float kx;
						float ky;
						float ktheat;
						T06_RF[2]=T06_init_RF[2];
						T06_LM[2]=T06_init_LM[2];
						T06_RB[2]=T06_init_RB[2];
						
						kx=(RLR_P2[0]-RLR_P1[0])/0.5f;
						ky=(RLR_P2[1]-RLR_P1[1])/0.5f;
						ktheat=(RLR_theat2-RLR_theat1)/0.5f;
						RLR_position(		0,RLR_P1[0]+kx*(RLR_phase-0.5f),	RLR_P1[1]+ky*(RLR_phase-0.5f),RLR_theat1+ktheat*(RLR_phase-0.5f)  );
					}					
				}
					
	//平移
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

void ChaBu(float T,float NUM,float h0,float hEnd,float n,float *dt,float *hNow)
{
	float dh;

	
	*dt = T/NUM;
	dh = (hEnd - h0)/NUM;

	*hNow = h0 + (n+1)*dh;
}
//action任务函数
void action_task(void *pvParameters)
{
	while(1)
	{
		if(flag_action_run_single==1||flag_action_run_continiue==1)//出现（单次/连续）运行触发信号
		{
			if(STA_ACTION == NORMAL)//正常行走模式
			{
				startUp_Or_End();//启动或停止					
				
				#if USE_T06_ideal
				#else				
				T06_init();//初始化末端位置	
				#endif				
				generate_Step_Vector();//生成平面步长向量
				
				generate_Initial_And_End_Position();//生成平面初末位置

				gain_Phase();//获得当前相位
			
//				printf("[%f,%f]",LRL_phase,RLR_phase);
				
				wether_Go_To_Next_Cycle();//判断是否进入下一周期

				wether_Change_State();//改变状态

				action();//计算出末端坐标

				flag_action_run_single=2;	
				
				vTaskDelay(1);
			}
			else if(STA_ACTION == TEST)//测试模式
			{
				RLR_high(0,0);		
				if(STA_TEST_TaiJiao == 1)//抬脚
				{
					for(ChaBu_n_TEST = 0;ChaBu_n_TEST<ChaBu_NUM_TEST;ChaBu_n_TEST++)
					{
						ChaBu(time_TEST_TaiJiao,ChaBu_NUM_TEST,0,h_TEST,ChaBu_n_TEST,&ChaBu_dt,&ChaBu_hNow);
						LRL_high(ChaBu_dt,ChaBu_hNow);
						flag_action_run_single=2;
						vTaskDelay(ChaBu_dt);
//						while(flag_action_run_single!=0)vTaskDelay(10);//等待运行结束
					}
					while(STA_TEST_TaiJiao == 1)vTaskDelay(10);;//停止
				}
				else if(STA_TEST_TaiJiao == 0)//落脚
				{

					STA_ChuDi_rm = 0;
					STA_ChuDi_lf = 0;
					STA_ChuDi_lb = 0;		
					
					for(ChaBu_n_TEST = 0;ChaBu_n_TEST<ChaBu_NUM_TEST;ChaBu_n_TEST++)
					{
						ChaBu(time_TEST_LuoJiao,ChaBu_NUM_TEST,h_TEST,0,ChaBu_n_TEST,&ChaBu_dt,&ChaBu_hNow);

						if  (STA_ChuDi_rm == 0)
						{
							T06_RM[2]=T06_init_RM[2]+ChaBu_hNow;
						}
						if  (STA_ChuDi_lf == 0)
						{
							T06_LF[2]=T06_init_LF[2]+ChaBu_hNow;
						}
						if  (STA_ChuDi_lb == 0) 
						{
							T06_LB[2]=T06_init_LB[2]+ChaBu_hNow;
						}
						
						flag_action_run_single=2;			
						vTaskDelay(ChaBu_dt);	

//						while(flag_action_run_single!=0)vTaskDelay(10);//等待运行结束
					}
					while(STA_TEST_TaiJiao == 0)vTaskDelay(10);//停止
				}
			}
		}
			vTaskDelay(10);
	}
}
















