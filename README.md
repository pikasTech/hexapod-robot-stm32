<img align="left" src="https://emojis.slackmojis.com/emojis/images/1563480763/5999/meow_party.gif" width="60" height="60"/>

# 来颗Star求求了

基于实时反馈和遗传算法的六足机器人全面步态优化系统设计。

完成了核心步态实时生成算法的设计，基于精确的运动学建模直接求解，不需要迭代计算，运算量小，可装载在嵌入式系统中。

机器人的每一步运动都由算法实时生成，可根据运动命令协调地进行任意方向任意速度平移旋转混合运动。

与此同时，步态根据外界反馈与可调参数实时更新，并基于遗传算法对步态进行迭代优化。

# 论文
[基于等效连杆模型的六足机器人行进姿态闭环控制_李昂_吉林大学学报工学版](%E5%9F%BA%E4%BA%8E%E7%AD%89%E6%95%88%E8%BF%9E%E6%9D%86%E6%A8%A1%E5%9E%8B%E7%9A%84%E5%85%AD%E8%B6%B3%E6%9C%BA%E5%99%A8%E4%BA%BA%E8%A1%8C%E8%BF%9B%E5%A7%BF%E6%80%81%E9%97%AD%E7%8E%AF%E6%8E%A7%E5%88%B6_%E6%9D%8E%E6%98%82.pdf)

# 演示视频
https://www.bilibili.com/video/BV17b411F7X8
![image](https://user-images.githubusercontent.com/88232613/129832474-5bf86442-a4d7-40c6-bcee-d166429032cf.png)

# 主要代码清单

## 逆运动学解算器与姿态PID控制
https://github.com/mimilib/hexapod-robot-stm32/blob/main/Main_freeRTOS/TASK/src/balance_task.c

## 三角步态生成器
https://github.com/mimilib/hexapod-robot-stm32/blob/main/Main_freeRTOS/TASK/src/action_task.c

## 基于遗传算法的步态参数优化
https://github.com/mimilib/hexapod-robot-stm32/blob/main/Main_freeRTOS/TASK/src/genetic_task.c

## 让机器人始终朝向发光体的视觉反馈控制
https://github.com/mimilib/hexapod-robot-stm32/blob/main/Main_freeRTOS/TASK/src/blobs_task.c
