#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#define M 100


double P_RF[3][1]={5.875,12.0,0};
double P_RM[3][1]={9,0,0};
double P_RB[3][1]={5.875,-12.0,0};
double P_LF[3][1]={-5.875,12.0,0};
double P_LM[3][1]={-9,0,0};
double P_LB[3][1]={-5.875,-12.0,0};

double P_RFx[3][1],P_RMx[3][1],P_RBx[3][1],P_LFx[3][1],P_LMx[3][1],P_LBx[3][1];
double P_RFxy[3][1],P_RMxy[3][1],P_RBxy[3][1],P_LFxy[3][1],P_LMxy[3][1],P_LBxy[3][1];



void rotate_x(double matrix2[3][1],double matrix[3][1],double theatx)
{

	double matrix1[3][3];//X轴旋转矩阵 
    int i,j,k;   

	matrix1[0][0]=1;
	matrix1[0][1]=0;
	matrix1[0][2]=0;
	matrix1[1][0]=0;
	matrix1[1][1]=cos(theatx);
	matrix1[1][2]=-sin(theatx);
	matrix1[2][0]=0;
	matrix1[2][1]=sin(theatx);
	matrix1[2][2]=cos(theatx);

    /*初始化matrix：*/
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

}


void rotate_y(double matrix2[3][1],double matrix[3][1],double theaty)
{

	double matrix1[3][3];//X轴旋转矩阵 
    int i,j,k;   

	matrix1[0][0]=cos(theaty);
	matrix1[0][1]=0;
	matrix1[0][2]=sin(theaty);
	matrix1[1][0]=0;
	matrix1[1][1]=1;
	matrix1[1][2]=0;
	matrix1[2][0]=-sin(theaty);
	matrix1[2][1]=0;
	matrix1[2][2]=cos(theaty);

    /*初始化matrix：*/
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


}

double l1[3][1];//向量X
double l1x[3][1];//X轴旋转后向量X'
double l1xy[3][1];//X轴旋转后向量X'



int main(void)
{
	double theatx;
	double theaty;
    int i,j,k;   


	printf("输入沿x轴旋转的角度和沿y轴旋转的角度"); 
	scanf("%lf,%lf",&theatx,&theaty);
	theatx=theatx/180.0*3.1415926;
	theaty=theaty/180.0*3.1415926;

	printf("%f,%f\n",theatx,theaty);

		rotate_x(P_RF,P_RFx,theatx);
		rotate_y(P_RFx,P_RFxy,theaty);
		
		rotate_x(P_RM,P_RMx,theatx);
		rotate_y(P_RMx,P_RMxy,theaty);

		rotate_x(P_RB,P_RBx,theatx);
		rotate_y(P_RBx,P_RBxy,theaty);
		
		rotate_x(P_LF,P_LFx,theatx);
		rotate_y(P_LFx,P_LFxy,theaty);
		
		rotate_x(P_LM,P_LMx,theatx);
		rotate_y(P_LMx,P_LMxy,theaty);
		
		rotate_x(P_LB,P_LBx,theatx);
		rotate_y(P_LBx,P_LBxy,theaty);		


	printf("output:\n"); 

	
    for(i=0;i<3;i++){
        for(j=0;j<1;j++){
            printf("%f ",P_RFxy[i][j]); 
        } 
        
    } 
	  printf("\n"); 
		
    for(i=0;i<3;i++){
        for(j=0;j<1;j++){
            printf("%f ",P_RMxy[i][j]); 
        } 
        
    } 
	  printf("\n"); 
		
    for(i=0;i<3;i++){
        for(j=0;j<1;j++){
            printf("%f ",P_RBxy[i][j]); 
        } 
        
    } 
	  printf("\n"); 
		
    for(i=0;i<3;i++){
        for(j=0;j<1;j++){
            printf("%f ",P_LFxy[i][j]); 
        } 
        
    } 
	
		  printf("\n"); 
    for(i=0;i<3;i++){
        for(j=0;j<1;j++){
            printf("%f ",P_LMxy[i][j]); 
        } 
        
    } 
	  printf("\n"); 
		
    for(i=0;i<3;i++){
        for(j=0;j<1;j++){
            printf("%f ",P_LBxy[i][j]); 
        } 
      
    } 
	  printf("\n"); 

}


