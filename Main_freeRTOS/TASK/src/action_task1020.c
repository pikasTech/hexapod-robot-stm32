#include "action_task.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "rtp_test_task.h"
#include "adc_task.h"
#include "sys.h"
#include "balance_task.h"
//添加外部头文件 PID
#include "pid.h"
/*声明外部PID变量来修改固定的摆动态下的3 和XX态下的5*/
extern PID  RF_DIS;
extern PID  RM_DIS;
extern PID  RB_DIS;
extern PID  LF_DIS;
extern PID  LM_DIS;
extern PID  LB_DIS;

/*声明部分LRL 和RLR腿高分别设置*/
void LRL_high_single(int time,float h1,float h2,float h3); //LRL顺序为LF RM LB
void RLR_high_single(int time,float h1,float h2,float h3); //RLR顺序为RF LM RB

int action_t;
int action_n=0;
float angle_test[6];

#define S10 0
#define S0 	1
#define S01 2
#define S1 	3

//伸展系数
double L_init=10;

//机身高度
double H_init=-15;

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
float LRL_R0=0.4f;

//过渡提前占空比

float LRL_R0f=0.5f;

//前申比例
float LRL_Rf=0.5f;

//前转比例
float LRL_Rz=0.5f;

//前申比例
float LRL_Rf_use=0.5f;

//前转比例
float LRL_Rz_use=0.5f;

//时间
long int LRL_t;
long int RLR_t;

//相
int LRL_S;
int RLR_S;
//相位
float LRL_phase;
float RLR_phase;

//周期
float LRL_T=2000;
float RLR_T=2000;
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
float LRL_O[3]; 
float RLR_O[3];
float LRL_O_use[3]; 
float RLR_O_use[3];
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

	(d5*L + d6*L*cos(t5) + d6*H*sin(t5))/(d6*d6*cos(t5)*cos(t5) + d6*d6*sin(t5)*sin(t5)			\
	+ d5*d5 + 2.0f*d5*d6*cos(t5));

	s4=	\
	
	(d5*H - d6*L*sin(t5) + d6*H*cos(t5))/(d6*d6*cos(t5)*cos(t5) + d6*d6*sin(t5)*sin(t5)			\
	+ d5*d5 + 2.0f*d5*d6*cos(t5));
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
	 T06_LF[2]=T06_init_LF[2]+h;	

	 T06_RM[2]=T06_init_RM[2]+h;

   T06_LB[2]=T06_init_LB[2]+h;	
	
		 vTaskDelay(time);
}
void LRL_high_single(int time, float h1,float h2,float h3)
{
   T06_LF[2]=T06_init_LF[2]+h1;	

	 T06_RM[2]=T06_init_RM[2]+h2;

   T06_LB[2]=T06_init_LB[2]+h3;	
	
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
	
	 T06_RF[2]=T06_init_RF[2]+h;	
	 T06_LM[2]=T06_init_LM[2]+h;
   T06_RB[2]=T06_init_RB[2]+h;	
	
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

void wetherClearOrder(void)
{
			int i;
			if(STA_STOP==1)for(i=0;i<3;i++)LRL_O_use[i]=0;
			else for(i=0;i<3;i++)LRL_O_use[i]=LRL_O[i];

			for(i=0;i<3;i++)RLR_O_use[i]=LRL_O_use[i];
}
				
void generateStepVector(void)
{
				//计算平移步长向量
				LRL_L[0]=LRL_O_use[0]*LRL_T/1000.0f;
				LRL_L[1]=LRL_O_use[1]*LRL_T/1000.0f;

				RLR_L[0]=LRL_O_use[0]*LRL_T/1000.0f;
				RLR_L[1]=LRL_O_use[1]*LRL_T/1000.0f;

				//计算旋转角步长
				
				LRL_theat=LRL_O_use[2]*LRL_T/1000.0f;
	
				RLR_theat=RLR_O_use[2]*RLR_T/1000.0f;				
				}
				
				void generateInitialAndEndPosition(void)	
				{
				if(STA_START==1)LRL_Rf_use=0,LRL_Rz_use=0;//启动时前伸比和前转比置1
				else LRL_Rf_use=LRL_Rf,LRL_Rz_use=LRL_Rz;
				
				//计算平移变换初末位置
				LRL_P1[0]=LRL_L[0]*LRL_Rf_use;
				LRL_P1[1]=LRL_L[1]*LRL_Rf_use;

				LRL_P2[0]=-LRL_L[0]*(1-LRL_Rf_use);
				LRL_P2[1]=-LRL_L[1]*(1-LRL_Rf_use);
				
				RLR_P1[0]=RLR_L[0]*LRL_Rf_use;
				RLR_P1[1]=RLR_L[1]*LRL_Rf_use;

				RLR_P2[0]=-RLR_L[0]*(1-LRL_Rf_use);
				RLR_P2[1]=-RLR_L[1]*(1-LRL_Rf_use);

				//计算旋转变换初末位置
				LRL_theat1=LRL_theat*LRL_Rz_use;
				LRL_theat2=-LRL_theat*(1-LRL_Rz_use);
				
				RLR_theat1=RLR_theat*LRL_Rz_use;
				RLR_theat2=-RLR_theat*(1-LRL_Rz_use);


				}					
				void gainPhase(void)
				{
					//获取当前时间
				LRL_t=systime-LRL_t0;
				
				//获取当前相位
				LRL_phase=LRL_t/LRL_T;
				RLR_phase=LRL_phase+0.5f;
					
}
void wetherGoToNextCycle()
			{	
								//进入下一周期
				if(STA_STAND==0&&LRL_phase>=1)
				{
						LRL_phase=0,LRL_t0=systime,STA_START=0;//结束启动周期，进入正常运行周期
				}
				//进入下一周期
				if(STA_STAND==0&&RLR_phase>=1)
				{
					RLR_phase-=1;
				}
			}
void startUpOrEnd(void)
{
					
									if(	(LRL_phase>=0.5f&&LRL_phase<=0.53f)			\
					||(RLR_phase>=0.5f&&RLR_phase<=0.53f)	)
				{
					//判断是否需要切换至静立状态
					if(	(LRL_O_use[0]>-0.05f&&LRL_O_use[0]<0.05f)			\
						&&(LRL_O_use[1]>-0.05f&&LRL_O_use[1]<0.05f)			\
						&&(LRL_O_use[2]>-0.005f&&LRL_O_use[2]<0.005f)			\
						&&(RLR_O_use[0]>-0.05f&&RLR_O_use[0]<0.05f)			\
						&&(RLR_O_use[1]>-0.05f&&RLR_O_use[1]<0.05f)			\
						&&(RLR_O_use[2]>-0.005f&&RLR_O_use[2]<0.005f)			\
						)STA_STAND=1;//切换至静立状态					
				}
				
				if(STA_STAND==1)//静立时检测是否需要启动
				{
					LRL_t0=systime;//静立;
					if(	(LRL_O_use[0]>-0.05f&&LRL_O_use[0]<0.05f)				\
							&&(LRL_O_use[1]>-0.05f&&LRL_O_use[1]<0.05f)			\
							&&(LRL_O_use[2]>-0.005f&&LRL_O_use[2]<0.005f)			\
							&&(RLR_O_use[0]>-0.05f&&RLR_O_use[0]<0.05f)			\
							&&(RLR_O_use[1]>-0.05f&&RLR_O_use[1]<0.05f)			\
							&&(RLR_O_use[2]>-0.005f&&RLR_O_use[2]<0.005f)			\
							);
					else STA_STAND=0,STA_START=1;//需要移动时启动
				}
				
								wetherClearOrder();	//判断是否停止
					
}					
void wetherChangeState(void)
{
								//计算相位临界值
				LRL_phase10=LRL_R0/4.0f;
				LRL_phase0=0.5f-LRL_phase10;


//				RLR_phase10=LRL_R0/4.0f;
//				RLR_phase0=0.5f-LRL_phase10;	
				//相切换
				if(LRL_phase>=0&&LRL_phase<LRL_phase10)LRL_S=S10;
				else if(LRL_phase>=LRL_phase10&&LRL_phase<LRL_phase0)LRL_S=S0;
				else if(LRL_phase>=LRL_phase0&&LRL_phase<LRL_phase01)LRL_S=S01;				
				else if(LRL_phase>=LRL_phase01&&LRL_phase<1)LRL_S=S1;				
				
				//相切换
				if(RLR_phase>=0&&RLR_phase<LRL_phase10)RLR_S=S10;
				else if(RLR_phase>=LRL_phase10&&RLR_phase<LRL_phase0)RLR_S=S0;
				else if(RLR_phase>=LRL_phase0&&RLR_phase<LRL_phase01)RLR_S=S01;				
				else if(RLR_phase>=LRL_phase01&&RLR_phase<1)RLR_S=S1;				
}
				
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
						float h1,h2,h3;                      
						kh=(0-LF_DIS.OUT)/(LRL_phase10*(1-LRL_R0f));
						h1=LF_DIS.OUT+kh* (LRL_phase-LRL_phase0);
						if(h1<=0)h1=0;
						
						kh=(0-RM_DIS.OUT)/(LRL_phase10*(1-LRL_R0f));
						h2=RM_DIS.OUT+kh* (LRL_phase-LRL_phase0);
						if(h2<=0)h2=0;
						
						kh=(0-LB_DIS.OUT)/(LRL_phase10*(1-LRL_R0f));
						h2=LB_DIS.OUT+kh* (LRL_phase-LRL_phase0);
						if(h3<=0)h3=0;
					
						LRL_high_single(0,h1,h2,h3);
						//LRL_high(0,h);//落腿	
					}
					//摆动相0
					else if(LRL_S==S0)
					{
						//根据末端初末位置生成直线轨迹
						float kx;
						float ky;
						float ktheat;
						kx=(LRL_P1[0]-LRL_P2[0])/(0.5f-2*LRL_phase10);
						ky=(LRL_P1[1]-LRL_P2[1])/(0.5f-2*LRL_phase10);
						ktheat=(LRL_theat1-LRL_theat2)/(0.5f-2*LRL_phase10);
						LRL_position(0,	LRL_P2[0]+kx*(LRL_phase-LRL_phase10),LRL_P2[1]+ky*(LRL_phase-LRL_phase10),LRL_theat2+ktheat*(LRL_phase-LRL_phase10));
					}
					//过度相10					
					else if(LRL_S==S10)
					{
						float kh;
						float h1,h2,h3;
						kh=(LF_DIS.OUT-0)/(LRL_phase10*(1-LRL_R0f));
						if(LRL_phase<LRL_phase10*LRL_R0f)h1=0;
						else	h1=0+kh*(LRL_phase-LRL_phase10*LRL_R0f);
						
							kh=(RM_DIS.OUT-0)/(LRL_phase10*(1-LRL_R0f));
						if(LRL_phase<LRL_phase10*LRL_R0f)h2=0;
						else	h2=0+kh*(LRL_phase-LRL_phase10*LRL_R0f);
						
							kh=(LB_DIS.OUT-0)/(LRL_phase10*(1-LRL_R0f));
						if(LRL_phase<LRL_phase10*LRL_R0f)h3=0;
						else	h3=0+kh*(LRL_phase-LRL_phase10*LRL_R0f);
						
							LRL_high_single(0,h1,h2,h3);
						
						//LRL_high(0,h);//落腿	
					}
					//支撑相1
					else if(LRL_S==S1)
					{
						//根据末端初末位置生成直线轨迹
						float kx;
						float ky;
						float ktheat;
						kx=(LRL_P2[0]-LRL_P1[0])/0.5f;
						ky=(LRL_P2[1]-LRL_P1[1])/0.5f;
						ktheat=(LRL_theat2-LRL_theat1)/0.5f;
						LRL_position(		0,LRL_P1[0]+kx*(LRL_phase-0.5f),	LRL_P1[1]+ky*(LRL_phase-0.5f),LRL_theat1+ktheat*(LRL_phase-0.5f)  );
					}					

					
					
					//过度相01
					if(RLR_S==S01)
					{
						float kh;
						float h1,h2,h3;
						kh=(0-RF_DIS.OUT)/(LRL_phase10*(1-LRL_R0f));
						h1=RF_DIS.OUT+kh* (RLR_phase-LRL_phase0);
						if(h1<=0)h1=0;
						kh=(0-LM_DIS.OUT)/(LRL_phase10*(1-LRL_R0f));
						h2=LM_DIS.OUT+kh* (RLR_phase-LRL_phase0);
						if(h2<=0)h2=0;
						kh=(0-RB_DIS.OUT)/(LRL_phase10*(1-LRL_R0f));
						h3=RB_DIS.OUT+kh* (RLR_phase-LRL_phase0);
						if(h3<=0)h3=0;
						
						RLR_high_single(0,h1,h2,h3);
						//RLR_high(0,h);//落腿	
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
					  float kh;
						float h1,h2,h3;
						kh=(RF_DIS.OUT-0)/(LRL_phase10*(1-LRL_R0f));
						if(RLR_phase<LRL_phase10*LRL_R0f)h1=0;
						else	h1=0+kh*(RLR_phase-LRL_phase10*LRL_R0f);
						
						kh=(LM_DIS.OUT-0)/(LRL_phase10*(1-LRL_R0f));
						if(RLR_phase<LRL_phase10*LRL_R0f)h2=0;
						else	h2=0+kh*(RLR_phase-LRL_phase10*LRL_R0f);
						
						kh=(RB_DIS.OUT-0)/(LRL_phase10*(1-LRL_R0f));
						if(RLR_phase<LRL_phase10*LRL_R0f)h3=0;
						else	h3=0+kh*(RLR_phase-LRL_phase10*LRL_R0f);
						
						RLR_high_single(0,h1,h2,h3);
						//RLR_high(0,h);//落腿	
					}
					//支撑相1					
					else if(RLR_S==S1)
					{
						//根据末端初末位置生成直线轨迹
						float kx;
						float ky;
						float ktheat;
						kx=(RLR_P2[0]-RLR_P1[0])/0.5f;
						ky=(RLR_P2[1]-RLR_P1[1])/0.5f;
						ktheat=(RLR_theat2-RLR_theat1)/0.5f;
						RLR_position(		0,RLR_P1[0]+kx*(RLR_phase-0.5f),	RLR_P1[1]+ky*(RLR_phase-0.5f),RLR_theat1+ktheat*(RLR_phase-0.5f)  );
					}					
				}
					
	//平移
}
				
//action任务函数
void action_task(void *pvParameters)
{
			while(1)
			{
			  T06_init();					//初始化末端位置

				startUpOrEnd();//启动或停止
				
			  generateStepVector();//生成步长向量
				
				generateInitialAndEndPosition();//生成初末位置

				gainPhase();//获得当前相位

				wetherGoToNextCycle();//判断是否进入下一周期
		  
				wetherChangeState();//改变状态

				action();//执行动作

				vTaskDelay(10);
			}
}
