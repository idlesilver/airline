//*************引用库*************//
    #include <StaticThreadController.h>
    #include <Thread.h>
    #include <ThreadController.h>

    #include <CheapStepper.h>

    #include <PS2X_lib.h>

    #include <Stepper.h>

    #include <Servo.h>

    #include <PID_v1.h>

    #include <MPU6050_tockn.h>
    
    #include <Wire.h>

//*************设置宏变量，针脚*************//
    #define pressures true          
    #define rumble true

    #define PS2_DAT  14 //13 
    #define PS2_CMD  15 //11 
    #define PS2_SEL  16 //10 
    #define PS2_CLK  17 //12 

    #define WHEEL_PWM_1 2
    #define WHEEL_PWM_2 3
    #define WHEEL_PWM_3 4
    #define WHEEL_PWM_4 5
    #define SB_PWM_YAW  6
    #define SB_PWM_SHOOT 7
    #define SERVO_PIN   8
    #define FRICTION_WHEEL 9


    #define WHEEL_SPEED_READ_1 A0
    #define WHEEL_SPEED_READ_2 A1
    #define WHEEL_SPEED_READ_3 A2
    #define WHEEL_SPEED_READ_4 A3

    #define WHEEL_IN1_1    30
    #define WHEEL_IN2_1    31
    #define WHEEL_IN1_2    32
    #define WHEEL_IN2_2    33
    #define WHEEL_IN1_3    34
    #define WHEEL_IN2_3    35
    #define WHEEL_IN1_4    36
    #define WHEEL_IN2_4    37

    #define STEPPER_YAW_1    22 //30
    #define STEPPER_YAW_2    23 //32
    #define STEPPER_YAW_3    24 //34
    #define STEPPER_YAW_4    25 //36
    #define STEPPER_SHOOT_1  26 //31
    #define STEPPER_SHOOT_2  27 //33
    #define STEPPER_SHOOT_3  28 //35
    #define STEPPER_SHOOT_4  29 //37

    #define SB_YAW_IN1 38
    #define SB_YAW_IN2 39
    #define SB_SHOOT_IN1 40
    #define SB_SHOOT_IN2 41

    #define use_PID false

//*************设置全局变量*************//
  //底座部分  
    int speed_x = 0;                   
    int speed_y = 0;
    double wheel_speed_1 = 0;             //每个轮子希望达到的速度
    double wheel_speed_2 = 0;
    double wheel_speed_3 = 0;
    double wheel_speed_4 = 0;
    double wheel_pwm_1 = 0;               //给每个轮子的pwm信号，接给H桥的enable口，经手PID控制
    double wheel_pwm_2 = 0;
    double wheel_pwm_3 = 0;
    double wheel_pwm_4 = 0;
    double wheel_current_speed_1 = 0;         //读到的每个轮子的当前转速
    double wheel_current_speed_2 = 0;
    double wheel_current_speed_3 = 0;
    double wheel_current_speed_4 = 0;
    const int rotating_speed = 130;

    const double wheel_pwm_change_unit = 1;       //轮速渐变用

    double Kp_wheel=0, Ki_wheel=0, Kd_wheel=0;      //PID用

    long  last_front_change = 0;                        //cache
    const int front_change_delay = 300;//ms            //切换方向的消抖延时
    int     front = 0;                      //0，1，2，3四个值，分别表示一个方向，按circle键依次切换正方向
    int     rotating = 0;                   //正数顺转，负数逆转，0不转，“优先度”要高于平移运动

  //云台部分
    float   angle_theta = 0;                //云台水平角度，这些都是target，current由mpu读取。PID控制，TODO:初始值由6轴传感器测定
    float   angle_alpha = 0;                //就是默认初始值偏移的角度，写给servo类时，用angle_alpha+angle_alpha_offset
    float   current_angle_theta = 0;        //由传感器测得的当前角度值
    float   current_angle_alpha = 0;

    bool    sb_turn_clockwise = false;              //给智障电机的旋转信号
    bool    sb_turn_counterclockwise = false;
    long    sweep_speed_level = 0;                  //云台旋转的速度挡位1，2，3（满速除以这个数值）
    long    sweep_speed_level_last_change_time = 0;
    const long  sweep_speed_level_delay = 300;//ms  //切换云台转速的挡位

    bool    stepper_move_clockwise = true;
    float   step_theta = 0;                         //用作储存中间变量,不用改
    const float   angle_theta_change_unit = 1;      //云台水平变化的角度，记得改
    const int     step_each_time = 2;               //stepper_yaw每次改变的步数，用于控制theta的精度
    const int     speedup_ratio = 2;                //yaw电机轴速度和云台真实轴速度的比值，不是加速比
    const float   angle_per_step = 5.625/8;
    const int     sb_yaw_speed = 255;

    const float     angle_alpha_change_unit = 1;    //云台仰角每次检测变化的角度，记得改
    const float     angle_alpha_offset = 90;        //就是中立位正负的角度
    const float     angle_alpha_max = 30;                 
    const float     angle_alpha_min = -15;

  //射击部分
    bool            shoot_once = false;         //不要once了       
    bool            shoot_dadada = false;
    bool            friction_wheel_on = false;  //摩擦轮转动标志
    const int       shoot_speed = 20;           //供弹电机转速
    const int       sb_shoot_speed = 100;       //供弹智障电机转速

  //手柄部分
    int stick_sensitive_val = 20;               //摇杆在中位会有数值波动，用sensitive_val来防抖 
    

//*************新建实例，初始化实例*************//
    Thread readPad = Thread();
    Thread servoUpdate = Thread();
    PS2X ps2x; // create PS2 Controller Class
        byte vibrate = 0;
        int ps2x_error = 0;
        void (*resetFunc)(void) = 0;
    Servo myservo;
    MPU6050 mpu6050(Wire);  // 新建一个mpu6050实例
    CheapStepper stepper_yaw(STEPPER_YAW_1,STEPPER_YAW_2,STEPPER_YAW_3,STEPPER_YAW_4);  
    CheapStepper stepper_shoot(STEPPER_SHOOT_1,STEPPER_SHOOT_2,STEPPER_SHOOT_3,STEPPER_SHOOT_4);  

    PID wheel_1(&wheel_current_speed_1, &wheel_pwm_1, &wheel_speed_1, Kp_wheel, Ki_wheel, Kd_wheel, DIRECT);
    PID wheel_2(&wheel_current_speed_2, &wheel_pwm_2, &wheel_speed_2, Kp_wheel, Ki_wheel, Kd_wheel, DIRECT);
    PID wheel_3(&wheel_current_speed_3, &wheel_pwm_3, &wheel_speed_3, Kp_wheel, Ki_wheel, Kd_wheel, DIRECT);
    PID wheel_4(&wheel_current_speed_4, &wheel_pwm_4, &wheel_speed_4, Kp_wheel, Ki_wheel, Kd_wheel, DIRECT);
//*************setup,loop主程序*************//
void setup(){
    //*************设置针脚模式*************//
        pinMode(PS2_DAT,OUTPUT);
        pinMode(PS2_CMD,OUTPUT);
        pinMode(PS2_SEL,OUTPUT);
        pinMode(PS2_CLK,OUTPUT);

        pinMode(WHEEL_PWM_1,OUTPUT);
        pinMode(WHEEL_PWM_2,OUTPUT);
        pinMode(WHEEL_PWM_3,OUTPUT);
        pinMode(WHEEL_PWM_4,OUTPUT);
        pinMode(SB_PWM_YAW,OUTPUT);
        pinMode(SB_PWM_SHOOT,OUTPUT);

        pinMode(WHEEL_IN1_1,OUTPUT);
        pinMode(WHEEL_IN2_1,OUTPUT);
        pinMode(WHEEL_IN1_2,OUTPUT);
        pinMode(WHEEL_IN2_2,OUTPUT);
        pinMode(WHEEL_IN1_3,OUTPUT);
        pinMode(WHEEL_IN2_3,OUTPUT);
        pinMode(WHEEL_IN1_4,OUTPUT);
        pinMode(WHEEL_IN2_4,OUTPUT);
        pinMode(SB_YAW_IN1,OUTPUT);
        pinMode(SB_YAW_IN2,OUTPUT);
        pinMode(SB_SHOOT_IN1,OUTPUT);
        pinMode(SB_SHOOT_IN2,OUTPUT);

        pinMode(FRICTION_WHEEL,OUTPUT);
        pinMode(SERVO_PIN,OUTPUT);

        pinMode(WHEEL_SPEED_READ_1,INPUT); //TODO:还没有设置读取函数，现在只有脚
        pinMode(WHEEL_SPEED_READ_1,INPUT);
        pinMode(WHEEL_SPEED_READ_1,INPUT);
        pinMode(WHEEL_SPEED_READ_1,INPUT);

        //servo和stepper在类中有pinmode初始化

    Serial.begin(9600);       //测试用
    //*************链接手柄*************//
        // delay(500);                //手柄配对延时
        ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
        // ps2x_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
        // if (ps2x_error == 0) Serial.print("Found Controller, configured successful ");
        // else Serial.println("there is an ps2x_error, but doesn't metter!");
        readPad.onRun(update_value_from_pad);
        readPad.setInterval(50);
    //*************初始化位置*************//
        // mpu_initial();
        servo_initial();                    //pitch轴回中
        servoUpdate.onRun(servo_control);
        servoUpdate.setInterval(10);
        // stepper_yaw_initial();              //设置yaw轴步进电机速度
        // stepper_shoot_initial();
        analogWrite(SB_PWM_SHOOT,0);
        analogWrite(SB_PWM_YAW,0);
    //*************PID控制*************//
        wheel_1.SetMode(AUTOMATIC);
        wheel_2.SetMode(AUTOMATIC);
        wheel_3.SetMode(AUTOMATIC);
        wheel_4.SetMode(AUTOMATIC);
    }
void loop(){ 
    // int last_time;
    // int now;
    // last_time = micros();
    //*************链接手柄*************//
        if (ps2x_error == 1){resetFunc();}
        if(readPad.shouldRun())
            readPad.run();
        // update_value_from_pad();
    //*************读取当前位置*************//
        // update_current_position();
    //*************车轮PID控制*************//
        speed_combine();
        if(use_PID){
            wheel_1.Compute();
            wheel_2.Compute();
            wheel_3.Compute();
            wheel_4.Compute();
            }else{
            wheel_pwm_without_PID();
            }
        motor_control();
    //*************舵机控制*************//
        // servo_control();
        if(servoUpdate.shouldRun())
            servoUpdate.run();
    //*************云台转向控制*************//
        sb_yaw_openloop_without_angle();
    //*************射弹控制*************//
        friction_wheel_run();
        sb_shoot_dadada();
    // Serial.print("time: ");
    // now = micros();
    // Serial.println(now-last_time);
    }

//*************手柄更新*************//
void ps2x_initial(){
    delay(1000);                //手柄配对延时
    ps2x_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    if (ps2x_error == 0) Serial.print("Found Controller, configured successful ");
    else Serial.println("there is an ps2x_error, but doesn't metter!");
}
void update_value_from_pad(){
    /* 从遥控手柄读取并更新目标值
     * output：
     *  speed_x
     *  speed_y
     *  rotating
     *  front
     *  angle_alpha
     *  angle_theta
     *  shoot_once
     *  shoot_dadada
     *忽略按键:
     *  start
     *  select
     *  PSB_L2(test_only now)
     *  PSB_R2(test_only now)
     *  PSB_L3
     *  PSB_R3
     *  Square
     *  Triangle(test_only)
     *  CROSS(test_only now)
     */
    float  step_alpha = 0;             //用作储存中间变量,不用改
    //******************读手柄数据******************//
        ps2x.read_gamepad(false, vibrate);
        vibrate = ps2x.Analog(PSAB_CROSS); //（X）键按多重，就震动多重，用来快速检测有没有连接上手柄
    //更新左边上下左右按键
      //上下键控制云台仰角
        if (ps2x.Button(PSB_PAD_UP))
        {
            if (angle_alpha >= angle_alpha_max)
                angle_alpha = angle_alpha_max;
            else
                angle_alpha += angle_alpha_change_unit;
            Serial.print("UP is pressed, ");
            Serial.print("angle_alpha is ");
            Serial.println(angle_alpha);
        }
        else if (ps2x.Button(PSB_PAD_DOWN))
        {
            if (angle_alpha <= angle_alpha_min)
                angle_alpha = angle_alpha_min;
            else
                angle_alpha -= angle_alpha_change_unit;
            Serial.print("DOWN is pressed, ");
            Serial.print("angle_alpha is ");
            Serial.println(angle_alpha);
        }
      //左右键控制是否顺逆时针旋转
        if (ps2x.Button(PSB_PAD_RIGHT))
        {   //TODO:吃得消的话，可以把按键的压力值变成旋转的速度
            rotating = -1;
            Serial.print("rotating is ");
            Serial.println(rotating);
        }
        else if (ps2x.Button(PSB_PAD_LEFT))
        {
            rotating = 1;
            Serial.print("rotating is ");
            Serial.println(rotating);
        }
        else
        {
            rotating = 0;
        }
    //按下circle键，改变front，即小车正方向
        if (ps2x.ButtonPressed(PSB_CIRCLE))
        {   //TODO:ButtonPressed本身就有消抖
            if (millis()-last_front_change >= front_change_delay){
                front = (front + 1) % 4;
                last_front_change = millis();
            Serial.print("Circle just pressed, front changed to: ");
            Serial.println(front);
            }
        }
    //按下square键，改变yaw轴旋转速度
        if (ps2x.ButtonPressed(PSB_SQUARE))
        {   //TODO:ButtonPressed本身就有消抖
            if (millis()-last_front_change >= front_change_delay){
                sweep_speed_level = (sweep_speed_level + 1) % 3 ;
                last_front_change = millis();
            Serial.print("Sweep speed is changed to: ");
            Serial.println(sweep_speed_level);
            }
        }
    //摇杆的值作为移动速度
        //测试用的串口显示：按L1或L2，输出摇杆值，用于检测
            if (ps2x.Button(PSB_R2))
            {  
                Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
                Serial.print(",");
                Serial.print(ps2x.Analog(PSS_LX), DEC);
                Serial.print(" | ");
                Serial.print(ps2x.Analog(PSS_RY), DEC);
                Serial.print(",");
                Serial.println(ps2x.Analog(PSS_RX), DEC);
            }
        //平移速度
            if (abs(ps2x.Analog(PSS_LX)-127) >= stick_sensitive_val){
                speed_x = map(ps2x.Analog(PSS_LX),0,255,-255,255);}      //FIXME: 摇杆的方向和数值关系，测一次之后可能改顺序
            else speed_x = 0;

            if (abs(ps2x.Analog(PSS_LY)-127) >= stick_sensitive_val){
                speed_y = map(ps2x.Analog(PSS_LY),0,255,255,-255);}      //结果要反向，因为pad上往下是大；这里可以用map，因为结果给PWM，只要整数
            else speed_y = 0;

            if (ps2x.Button(PSB_TRIANGLE)){
                Serial.print("speed_x is: ");
                Serial.println(speed_x);
                Serial.print("speed_y is: ");
                Serial.println(speed_y);
            }
        //云台仰角目标改变
            step_alpha = -(ps2x.Analog(PSS_RY)/255.0*angle_alpha_change_unit*4-angle_alpha_change_unit*2); //取了一个负号，因为摇杆方向不对
            if (abs(ps2x.Analog(PSS_RY) -127) >= stick_sensitive_val){     
                if (step_alpha >0 && angle_alpha >= angle_alpha_max) angle_alpha = angle_alpha_max;
                else if (step_alpha <0 && angle_alpha <= angle_alpha_min) angle_alpha = angle_alpha_min;
                else angle_alpha += step_alpha;                 //这里反向，因为pad stick往下数字更大
                Serial.print("angle_alpha is: ");
                Serial.println(angle_alpha);
            }
        //云台水平目标角度
            step_theta = ps2x.Analog(PSS_RX)/255.0*angle_theta_change_unit*4-angle_theta_change_unit*2; //用了255.0防止第一个除法两个整数相除变成0；等价于有float的map(ps2x.Analog(PSS_RX),0,255,-angle_theta_change_unit*2,angle_theta_change_unit*2)
            if (abs(ps2x.Analog(PSS_RX)-127) >= stick_sensitive_val) {
                angle_theta += step_theta;
                Serial.print("angle_theta is: ");
                Serial.println(angle_theta);
            }
            if (abs(ps2x.Analog(PSS_RX)-127) >= stick_sensitive_val) {//给智障电机的旋转信号
                if(ps2x.Analog(PSS_RX)-127 > 0){sb_turn_clockwise = true;sb_turn_counterclockwise = false;}
                else if(ps2x.Analog(PSS_RX)-127 < 0){sb_turn_clockwise = false;sb_turn_counterclockwise = true;} //FIXME:yaw电机只能向一个方向转！！
                Serial.print("CW, CCW: ");
                Serial.print(sb_turn_clockwise);
                Serial.print(" | ");
                Serial.println(sb_turn_counterclockwise);
            }else{
                sb_turn_clockwise =false;
                sb_turn_counterclockwise =false;
            }
    //射击模块
        if (ps2x.ButtonPressed(PSB_L1)){
            friction_wheel_on = !friction_wheel_on;
            Serial.print("friction_wheel_on is: ");
            Serial.println(friction_wheel_on);
        }
        if (ps2x.Button(PSB_R1)){
            shoot_dadada = true;
            // Serial.print("shoot_dadada: ");
            // Serial.println(shoot_dadada);
        }else{
            shoot_dadada = false;
        }
    delay(50);      //FIXME:之后用多线程，这个就在线程delay中做掉
}
//*************车轮控制*************//
void speed_combine(){
    /* 把speed_x，y和front变成四个轮子的speed
     * 对应的数值正负表示旋转方向
     * input:
     *  speed_x
     *  speed_y
     * output：
     *   wheel_speed_1
     *   wheel_speed_2
     *   wheel_speed_3
     *   wheel_speed_4
     * */
    int wheel_direction_1 = 1;
    int wheel_direction_2 = 1;
    int wheel_direction_3 = 1;
    int wheel_direction_4 = 1;
    if(rotating == 1){
        wheel_direction_1 = 1;
        wheel_direction_2 = 1;
        wheel_direction_3 = 1;
        wheel_direction_4 = 1;
        wheel_speed_1 = rotating_speed * wheel_direction_1 ;
        wheel_speed_2 = rotating_speed * wheel_direction_2 ;
        wheel_speed_3 = rotating_speed * wheel_direction_3 ;
        wheel_speed_4 = rotating_speed * wheel_direction_4 ;
    }else if(rotating == -1){
        wheel_direction_1 = -1;
        wheel_direction_2 = -1;
        wheel_direction_3 = -1;
        wheel_direction_4 = -1;
        wheel_speed_1 = rotating_speed * wheel_direction_1 ;
        wheel_speed_2 = rotating_speed * wheel_direction_2 ;
        wheel_speed_3 = rotating_speed * wheel_direction_3 ;
        wheel_speed_4 = rotating_speed * wheel_direction_4 ;
    }else{
        switch (front){
            case 0:
                wheel_direction_1 =  1;
                wheel_direction_2 = -1;
                wheel_direction_3 = -1;
                wheel_direction_4 =  1;
                break;
            case 1:
                wheel_direction_1 =  1;
                wheel_direction_2 =  1;
                wheel_direction_3 = -1;
                wheel_direction_4 = -1;
                break;
            case 2:
                wheel_direction_1 = -1;
                wheel_direction_2 =  1;
                wheel_direction_3 =  1;
                wheel_direction_4 = -1;
                break;
            case 3:
                wheel_direction_1 = -1;
                wheel_direction_2 = -1;
                wheel_direction_3 =  1;
                wheel_direction_4 =  1;
                break;
            default:
                break;
        }
        wheel_speed_1 = speed_x * wheel_direction_1 ;
        wheel_speed_2 = speed_x * wheel_direction_2 ;
        wheel_speed_3 = speed_x * wheel_direction_3 ;
        wheel_speed_4 = speed_x * wheel_direction_4 ;
        switch (front){
            case 0:
                wheel_direction_1 = -1;// 1;
                wheel_direction_2 = -1;//-1;
                wheel_direction_3 =  1;//-1;
                wheel_direction_4 =  1;// 1;
                break;
            case 1:
                wheel_direction_1 =  1;// 1;
                wheel_direction_2 = -1;// 1;
                wheel_direction_3 = -1;//-1;
                wheel_direction_4 =  1;//-1;
                break;
            case 2:
                wheel_direction_1 =  1;//-1;
                wheel_direction_2 =  1;// 1;
                wheel_direction_3 = -1;// 1;
                wheel_direction_4 = -1;//-1;
                break;
            case 3:
                wheel_direction_1 = -1;//-1;
                wheel_direction_2 =  1;//-1;
                wheel_direction_3 =  1;// 1;
                wheel_direction_4 = -1;// 1;
                break;
            default:
                break;
        }
        wheel_speed_1 += speed_y * wheel_direction_1 ;
        wheel_speed_2 += speed_y * wheel_direction_2 ;
        wheel_speed_3 += speed_y * wheel_direction_3 ;
        wheel_speed_4 += speed_y * wheel_direction_4 ;

        if (abs(wheel_speed_1) >255){
            if(wheel_speed_1>0){wheel_speed_1 = 255;}
            else{wheel_speed_1 = -255;}
        }else{wheel_speed_1 = wheel_speed_1;}

        if (abs(wheel_speed_2) >255){
            if(wheel_speed_2>0){wheel_speed_2 = 255;}
            else{wheel_speed_2 = -255;}
        }else{wheel_speed_2 = wheel_speed_2;}
        
        if (abs(wheel_speed_3) >255){
            if(wheel_speed_3>0){wheel_speed_3 =  255;}
            else{wheel_speed_3 = -255;}
        }else{wheel_speed_3 = wheel_speed_3;}

        if (abs(wheel_speed_4) >255){
            if(wheel_speed_4>0){wheel_speed_4 =  255;}
            else{wheel_speed_4 = -255;}
        }else{wheel_speed_4 = wheel_speed_4;}
    }
}
void wheel_pwm_without_PID(){
    /* 直接把wheel_speed变成pwm没有pid控制,逐渐增加，因为直接给255的话，4个电机不能一起转（学生电源）
     * input:
     *  wheel_speed_[1-4]
     * output:
     *  wheel_pwm_[1-4]
     */
    if(wheel_speed_1 == 0){wheel_pwm_1 =0;}
    else if(wheel_pwm_1 > wheel_speed_1){wheel_pwm_1 -= wheel_pwm_change_unit;}
    else if(wheel_pwm_1 < wheel_speed_1){wheel_pwm_1 += wheel_pwm_change_unit;}

    if(wheel_speed_2 == 0){wheel_pwm_2 =0;}
    else if(wheel_pwm_2 > wheel_speed_2){wheel_pwm_2 -= wheel_pwm_change_unit;}
    else if(wheel_pwm_2 < wheel_speed_2){wheel_pwm_2 += wheel_pwm_change_unit;}

    if(wheel_speed_3 == 0){wheel_pwm_3 =0;}
    else if(wheel_pwm_3 > wheel_speed_3){wheel_pwm_3 -= wheel_pwm_change_unit;}
    else if(wheel_pwm_3 < wheel_speed_3){wheel_pwm_3 += wheel_pwm_change_unit;}

    if(wheel_speed_4 == 0){wheel_pwm_4 =0;}
    else if(wheel_pwm_4 > wheel_speed_4){wheel_pwm_4 -= wheel_pwm_change_unit;}
    else if(wheel_pwm_4 < wheel_speed_4){wheel_pwm_4 += wheel_pwm_change_unit;}
    
    //测试用
    if (ps2x.Button(PSB_L2)){
        Serial.print("wheel speed: ");
        Serial.print(wheel_speed_1);
        Serial.print(" | ");
        Serial.print(wheel_speed_2);
        Serial.print(" | ");
        Serial.print(wheel_speed_3);
        Serial.print(" | ");
        Serial.println(wheel_speed_4);
        Serial.print("wheel pwm:   ");
        Serial.print(wheel_pwm_1);
        Serial.print(" | ");
        Serial.print(wheel_pwm_2);
        Serial.print(" | ");
        Serial.print(wheel_pwm_3);
        Serial.print(" | ");
        Serial.println(wheel_pwm_4);
        }
}
void motor_control(){
    /* 根据wheel_pwm的正负给方向，根据绝对值给大小
     * 传入H桥
     * input:
     *  wheel_pwm_[1-4]
     * output：
     *  WHEEL_IN[1-2]_[1-4]
     *  WHEEL_PWM_[1-4]
     */
    if (wheel_pwm_1 > 0){
        digitalWrite(WHEEL_IN1_1,HIGH);
        digitalWrite(WHEEL_IN2_1,LOW);
        }else if (wheel_pwm_1 < 0){
        digitalWrite(WHEEL_IN1_1,LOW);
        digitalWrite(WHEEL_IN2_1,HIGH);
        }else{
        digitalWrite(WHEEL_IN1_1,LOW);
        digitalWrite(WHEEL_IN2_1,LOW);
        }
    if (wheel_pwm_2 > 0){
        digitalWrite(WHEEL_IN1_2,HIGH);
        digitalWrite(WHEEL_IN2_2,LOW);
        }else if (wheel_pwm_2 < 0){
        digitalWrite(WHEEL_IN1_2,LOW);
        digitalWrite(WHEEL_IN2_2,HIGH);
        }else{
        digitalWrite(WHEEL_IN1_2,LOW);
        digitalWrite(WHEEL_IN2_2,LOW);
        }
    if (wheel_pwm_3 > 0){
        digitalWrite(WHEEL_IN1_3,HIGH);
        digitalWrite(WHEEL_IN2_3,LOW);
        }else if (wheel_pwm_3 < 0){
        digitalWrite(WHEEL_IN1_3,LOW);
        digitalWrite(WHEEL_IN2_3,HIGH);
        }else{
        digitalWrite(WHEEL_IN1_3,LOW);
        digitalWrite(WHEEL_IN2_3,LOW);
        }
    if (wheel_pwm_4 > 0){
        digitalWrite(WHEEL_IN1_4,HIGH);
        digitalWrite(WHEEL_IN2_4,LOW);
        }else if (wheel_pwm_4 < 0){
        digitalWrite(WHEEL_IN1_4,LOW);
        digitalWrite(WHEEL_IN2_4,HIGH);
        }else{
        digitalWrite(WHEEL_IN1_4,LOW);
        digitalWrite(WHEEL_IN2_4,LOW);
        }
    analogWrite(WHEEL_PWM_1,abs(wheel_pwm_1));
    analogWrite(WHEEL_PWM_2,abs(wheel_pwm_2));
    analogWrite(WHEEL_PWM_3,abs(wheel_pwm_3));
    analogWrite(WHEEL_PWM_4,abs(wheel_pwm_4));
}
//*************云台指向*************//
void mpu_initial(){
    /* 初始化mpu */
    Wire.begin();                       // 开启 I2C 总线
    mpu6050.begin();                    // 开启mpu6050
    mpu6050.calcGyroOffsets(true);      // 计算初始位置
    mpu6050.update();
    current_angle_alpha = mpu6050.getGyroAngleY();  //初始化位置
    current_angle_theta = mpu6050.getGyroAngleZ();
    angle_alpha = mpu6050.getGyroAngleY();
    angle_theta = mpu6050.getGyroAngleZ();
}
void update_current_position(){
    /* 更新当前位置 */
    int timer =0;
    mpu6050.update();              // 更新当前位置
    if (millis() - timer > 500) {         // 每500ms更新一次当前位置
        // Serial.print(mpu6050.getGyroAngleX());
        // Serial.print(" | ");
        // Serial.print(mpu6050.getGyroAngleY());
        // Serial.print(" | ");
        // Serial.println(mpu6050.getGyroAngleZ());
        current_angle_alpha = mpu6050.getGyroAngleY();  //这个轴好像都不用
        current_angle_theta = mpu6050.getGyroAngleZ();
        // Serial.print("current_angle_theta: ");
        Serial.println(mpu6050.getGyroAngleZ());
        timer = millis();
    }
}
//*************舵机指向*************//
void servo_initial(){
    myservo.attach(SERVO_PIN,500,2500);
    myservo.write(angle_alpha_offset);                  //pitch轴回中
}
void servo_control(){
    static long lasttime = 0;
    myservo.writeMicroseconds(map(int(angle_alpha+angle_alpha_offset),0,180,500,2500));
    // Serial.println((map(int(angle_alpha+angle_alpha_offset),0,180,500,2500)));
    // Serial.println(micros() - lasttime);
    lasttime = micros();
}
void servo_control_raw(){
    digitalWrite(SERVO_PIN,HIGH);
    delayMicroseconds(map(int(angle_alpha+angle_alpha_offset),0,180,500,2500));
    digitalWrite(SERVO_PIN,LOW);
}
//*************云台步进电机*************//
void stepper_yaw_initial(){
    stepper_yaw.setRpm(20);
    stepper_yaw.stop();
}
void stepper_yaw_with_angle(){//不知道库里的函数能不能直接用，这边加速齿轮会比较难算
    stepper_yaw.run();
    if(angle_theta - current_angle_theta > 0){stepper_move_clockwise = true;}
    else if(angle_theta - current_angle_theta <0){stepper_move_clockwise = false;}
    stepper_yaw.newMoveToDegree(stepper_move_clockwise,angle_theta);
}
void stepper_yaw_steps(){//这样一定能转到想转的位置,但是每次更新几步是个问题
    if (int(angle_theta - current_angle_theta) > 1){//化成int，防止两个float相减不为0
        if(angle_theta - current_angle_theta > 0){stepper_move_clockwise = true;}        //FIXME:不知道方向对不对，可能还大于小于号
        else if(angle_theta - current_angle_theta < 0){stepper_move_clockwise = false;}
        stepper_yaw.newMoveDegrees(stepper_move_clockwise,int(angle_theta - current_angle_theta)*speedup_ratio);//讲道理这里不用ratio也可以
    }else{
        stepper_yaw.stop();
    }
    stepper_yaw.run();
}
void stepper_yaw_steps_openloop(){//这样一定能转到想转的位置,但是每次更新几步是个问题
    int signal = 0;
    if (int(abs(angle_theta - current_angle_theta)) > 1){//化成int，防止两个float相减不为0
        if(angle_theta - current_angle_theta > 0){
            stepper_move_clockwise = true;           //FIXME:不知道方向对不对，可能还大于小于号
            signal = 1;
        }        
        else if(angle_theta - current_angle_theta < 0){
            stepper_move_clockwise = false;
            signal = -1;
            }
        stepper_yaw.newMoveDegrees(stepper_move_clockwise,30);//讲道理这里不用ratio也可以
        current_angle_theta += signal * angle_per_step * speedup_ratio;
        stepper_yaw.run();
        Serial.print("current & target: ");
        Serial.print(current_angle_theta);
        Serial.print(" | ");
        Serial.println(angle_theta);
    }else{
        stepper_yaw.stop();
    }
}
void sb_yaw_openloop(){//好像不行，可能是因为current更新太快，没有旋转
    int signal = 0;
    if (int(abs(angle_theta - current_angle_theta)) > 1){//化成int，防止两个float相减不为0
        if(angle_theta - current_angle_theta > 0){
            digitalWrite(SB_YAW_IN1,HIGH);
            digitalWrite(SB_YAW_IN2,LOW);
            signal = 1;
        }        
        else if(angle_theta - current_angle_theta < 0){
            digitalWrite(SB_YAW_IN1,LOW);
            digitalWrite(SB_YAW_IN2,HIGH);
            signal = -1;
        }
        current_angle_theta += signal * angle_per_step * speedup_ratio;
        Serial.print("current & target: ");
        Serial.print(current_angle_theta);
        Serial.print(" | ");
        Serial.println(angle_theta);
    }else{
        digitalWrite(SB_YAW_IN1,LOW);
        digitalWrite(SB_YAW_IN2,LOW);
    }
}
void sb_yaw_openloop_without_angle(){
    int signal = 0;
    analogWrite(SB_PWM_YAW,sb_yaw_speed-sweep_speed_level*60);//FIXME:调整最后的数字来调整yaw轴档位
    if(sb_turn_clockwise && !sb_turn_counterclockwise){
        digitalWrite(SB_YAW_IN1,HIGH);
        digitalWrite(SB_YAW_IN2,LOW);
    }else if(!sb_turn_clockwise && sb_turn_counterclockwise){
        digitalWrite(SB_YAW_IN1,LOW);
        digitalWrite(SB_YAW_IN2,HIGH);
    }else{
        digitalWrite(SB_YAW_IN1,LOW);
        digitalWrite(SB_YAW_IN2,LOW);
    }
}
//*************射弹控制*************//
void stepper_shoot_initial(){
    stepper_shoot.setRpm(shoot_speed);
    stepper_shoot.stop();
}
void stepper_shoot_dadada_run(){
    if(friction_wheel_on && shoot_dadada){
        if(stepper_shoot.getStepsLeft()==0){
            Serial.println("shoot!!!");
            stepper_shoot.newMoveDegrees(true,30);
            stepper_shoot.run();
        }else{
            stepper_shoot.run();
        }
    }else{
        Serial.println("no shoot");
        stepper_shoot.stop();
    }
}
void stepper_shoot_dadada_run_no_stop(){
    if(friction_wheel_on){
        if(shoot_dadada && stepper_shoot.getStepsLeft()==0){
            Serial.println("new shoot!!!");
            stepper_shoot.newMoveDegrees(true,30);
            stepper_shoot.run();
        }else if(stepper_shoot.getStepsLeft()!=0){
            Serial.println("dadadadadadadadada");
            stepper_shoot.run();
        }else{
            Serial.println("-------peace------");
            stepper_shoot.stop();
        }
    }else{
        stepper_shoot.stop();
    }
}
void sb_shoot_dadada(){
    if(friction_wheel_on && shoot_dadada){
        analogWrite(SB_PWM_SHOOT,sb_shoot_speed);
        // digitalWrite(SB_SHOOT_IN1,sb_shoot_speed);
    }else{
        analogWrite(SB_PWM_SHOOT,0);
        // digitalWrite(SB_SHOOT_IN1,0);
    }
}
void friction_wheel_run(){
    if(friction_wheel_on){
        digitalWrite(FRICTION_WHEEL,HIGH);
        // Serial.println("friction_wheel: on");
    }else{
        digitalWrite(FRICTION_WHEEL,LOW);
        // Serial.println("friction_wheel: off");
    }
}