/* 首先include好多库
 * 然后建立全局变量
 * setup里面就只有initial函数
 * loop里面用多线程，或者自己写micro()计时的interval
 * DC电机用PID控制，编码盘的读取估计用中断，四个电机可能要用很多块板
 * 舵机用官方库，懒了
 * stepper用cheapstepper，比较准，速度PID控制；较准还要研究一下
 * 摩擦轮电机只有on/off状态，once和dadada只有时间不一样
 * 拨弹轮同理
 * mpu6050的库好好研究，sample_time选好，信息交互做好
 * 遥控器的数据处理基本按照之·前的来
 */