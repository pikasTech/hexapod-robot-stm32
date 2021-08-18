#include<stdio.h>
#include<math.h>

void main()
{
double x,y,z,t1,t2,t3,t4,t5;
double pi=3.1416;
t1=0;
t2=pi/3.0;
t3=pi/3.0;
t4=pi/3.0;
t5=pi/3.0;

x=15*cos(t1)*cos(t2) - 5*sin(t1)*sin(t3) - 15*cos(t5)*(cos(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) + cos(t1)*sin(t2)*sin(t4)) + 15*sin(t5)*(sin(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t2)*cos(t3)) - cos(t1)*cos(t4)*sin(t2)) - (15*cos(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t2)*cos(t3)))/2 - (15*cos(t1)*sin(t2)*sin(t4))/2 + 5*cos(t1)*cos(t2)*cos(t3);
y=15*cos(t2)*sin(t1) + 5*cos(t1)*sin(t3) + 15*cos(t5)*(cos(t4)*(cos(t1)*sin(t3) + cos(t2)*cos(t3)*sin(t1)) - sin(t1)*sin(t2)*sin(t4)) - 15*sin(t5)*(sin(t4)*(cos(t1)*sin(t3) + cos(t2)*cos(t3)*sin(t1)) + cos(t4)*sin(t1)*sin(t2)) + (15*cos(t4)*(cos(t1)*sin(t3) + cos(t2)*cos(t3)*sin(t1)))/2 - (15*sin(t1)*sin(t2)*sin(t4))/2 + 5*cos(t2)*cos(t3)*sin(t1);
z=15*sin(t2) + 5*cos(t3)*sin(t2) + (15*cos(t2)*sin(t4))/2 + 15*cos(t5)*(cos(t2)*sin(t4) + cos(t3)*cos(t4)*sin(t2)) + 15*sin(t5)*(cos(t2)*cos(t4) - cos(t3)*sin(t2)*sin(t4)) + (15*cos(t3)*cos(t4)*sin(t2))/2;

printf("%f,%f,%f",x,y,z);
}