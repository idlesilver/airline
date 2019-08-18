//*************引用库*************//
    #include <StaticThreadController.h>
    #include <Thread.h>
    #include <ThreadController.h>

    #include <CheapStepper.h>

    #include <PS2X_lib.h>

    #include <Stepper.h>

    #include <Servo.h>

    #include <PID_v1.h>

//*************设置宏变量，针脚*************//
    #define PS2_DAT  14 //13 
    #define PS2_CMD  15 //11 
    #define PS2_SEL  16 //10 
    #define PS2_CLK  17 //12 

    #define pressures true          
    #define rumble true

//*************设置全局变量*************//
    volatile int speed_x = 0;                   //水平移动速度
    volatile int speed_y = 0;
    volatile float angle_theta = 0;             //云台水平角度，PID控制，TODO:初始值由6轴传感器测定
    volatile float angle_alpha = 70;            //FIXME:舵机启动时的水平位置，记得改
    volatile bool shoot_once = 0;               //TODO:没注释的都还没设置update
    volatile bool shoot_dadada = 0;
    volatile int front = 0;                     //0，1，2，3四个值，分别表示一个方向，按circle键依次切换正方向
    volatile int rotating = 0;                  //正数顺转，负数逆转，0不转，“优先度”要高于平移运动

    float angle_theta_change_unit = 1.0;        //FIXME:云台水平变化的角度，记得改(可能把angle相关的改成float)
    float step_theta = 0;                       //用作储存中间变量,不用改
    int   speedup_ratio = 5;                    //云台齿轮组加速比

    float angle_alpha_change_unit = 0.5;        //FIXME:云台仰角每次检测变化的角度，记得改(可能把angle相关的改成float)
    float step_alpha = 0;                       //用作储存中间变量,不用改
    float angle_alpha_max = 90;                 //FIXME: 舵机的俯仰角限制范围，记得改
    float angle_alpha_min = 45;

    long  last_front_change = 0;                //cache
    long  front_change_delay = 300;//ms         //切换方向的消抖延时

    int stick_sensitive_val = 20;           //摇杆在中位会有数值波动，用sensitive_val来防抖 

//*************新建实例，初始化实例*************//
    PS2X ps2x; // create PS2 Controller Class
        byte vibrate = 0;
        int ps2x_error = 0;
        void (*resetFunc)(void) = 0;
//*************setup,loop主程序*************//
void setup(){
    Serial.begin(115200);       //测试用
    //*************链接手柄*************//
        delay(1000);                //手柄配对延时
        ps2x_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
        if (ps2x_error == 0)    {Serial.print("Found Controller, configured successful ");}
        else    {Serial.println("there is an ps2x_error, but doesn't metter!");}
    }
void loop(){
    if (ps2x_error == 1){resetFunc();}
    update_value_from_pad();
    
    }

void update_value_from_pad()
{
    /*忽略按键:
     *  start
     *  select
     *  PSB_L2(test_only now)
     *  PSB_R2(test_only now)
     *  PSB_L3
     *  PSB_R3
     *  Square
     *  Triangle
     *  CROSS(test_only now)
     */
    //******************读手柄数据******************//
        ps2x.read_gamepad(false, vibrate);
        vibrate = ps2x.Analog(PSAB_CROSS); //（X）键按多重，就震动多重，用来快速检测有没有连接上手柄
    //更新左边上下左右按键
      //上下键控制云台仰角 FIXME:真的需要这个功能？？？？？
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
            rotating = 1;
            Serial.print("rotating is ");
            Serial.println(rotating);
        }
        else if (ps2x.Button(PSB_PAD_LEFT))
        {
            rotating = -1;
            Serial.print("rotating is ");
            Serial.println(rotating);
        }
        else
        {
            rotating = 0;
        }
    //按下circle键，改变front，即小车正方向
        if (ps2x.ButtonPressed(PSB_CIRCLE))
        {   //TODO:这里可能需要一个消抖。。。
            if (millis()-last_front_change >= front_change_delay){
                front = (front + 1) % 4;
                last_front_change = millis();
            Serial.println("Circle just pressed, front changed to: ");
            Serial.println(front);
            }
        }
    //摇杆的值作为移动速度
        //测试用的串口显示：按L1或L2，输出摇杆值，用于检测
            if (ps2x.Button(PSB_L2) || ps2x.Button(PSB_R2))
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
            step_alpha = ps2x.Analog(PSS_RY)/255.0*angle_alpha_change_unit*4-angle_alpha_change_unit*2; //等价于有float的map(ps2x.Analog(PSS_RX),0,255,-angle_theta_change_unit*2,angle_theta_change_unit*2)
            // if (ps2x.Analog(PSS_RY) >= stick_sensitive_val_up || ps2x.Analog(PSS_RY) <= stick_sensitive_val_down){     //TODO: 前面变成float后，可以做摇杆位置——抬升速度的关联，但估计要用tanh函数，手柄不够灵敏
            if (abs(ps2x.Analog(PSS_RY) -127) >= stick_sensitive_val){     //TODO: 前面变成float后，可以做摇杆位置——抬升速度的关联，但估计要用tanh函数，手柄不够灵敏
                if (angle_alpha <= angle_alpha_min-0.01) angle_alpha = angle_alpha_min;
                else if (angle_alpha >= angle_alpha_max+0.01) angle_alpha = angle_alpha_max;
                else angle_alpha -= step_alpha;                 //这里反向，因为pad stick往下数字更大
                Serial.print("angle_alpha is: ");
                Serial.println(angle_alpha);
            }
        //云台水平目标角度
            step_theta = ps2x.Analog(PSS_RX)/255.0*angle_theta_change_unit*4-angle_theta_change_unit*2; //用了255.0防止第一个除法两个整数相除变成0
            if (abs(ps2x.Analog(PSS_RX)-127) >= stick_sensitive_val) {    //TODO: 前面变成float后，可以做摇杆位置——抬升速度的关联，但估计要用tanh函数，手柄不够灵敏
                angle_theta += step_theta;
                Serial.print("angle_theta is: ");
                Serial.println(angle_theta);
            }



    //*************暂时没用到的*************//
        // if (ps2x.NewButtonState()){will be TRUE if any button changes state (on to off, or off to on)
        //       if(ps2x.Button(PSB_L3))
        //         Serial.println("L3 pressed");
        //       if(ps2x.Button(PSB_R3))
        //         Serial.println("R3 pressed");
        //       if(ps2x.Button(PSB_L2))
        //         Serial.println("L2 pressed");
        //       if(ps2x.Button(PSB_R2))
        //         Serial.println("R2 pressed");
        // }
        // if(ps2x.Button(PSB_TRIANGLE))
        //     Serial.println("Triangle pressed");
        // if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
        //   Serial.println("X just changed");
        // if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
        //   Serial.println("Square just released");

    delay(50);      //FIXME:之后用多线程，这个就在线程delay中做掉
}