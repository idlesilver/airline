int wheel_pwm_1;
int wheel_pwm_2;
int wheel_pwm_3;
int wheel_pwm_4;
int speed_x;
int speed_y;
int rotating_speed;
int rotating;
int front;
void setup(){

}
void loop(){

}
void speed_combine(){
    int wheel_direction_1 = 1;
    int wheel_direction_2 = 1;
    int wheel_direction_3 = 1;
    int wheel_direction_4 = 1;
    if(rotating == 1){
        wheel_direction_1 = 1;
        wheel_direction_2 = 1;
        wheel_direction_3 = 1;
        wheel_direction_4 = 1;
        wheel_pwm_1 = rotating_speed * wheel_direction_1 ;
        wheel_pwm_2 = rotating_speed * wheel_direction_2 ;
        wheel_pwm_3 = rotating_speed * wheel_direction_3 ;
        wheel_pwm_4 = rotating_speed * wheel_direction_4 ;
    }else if(rotating == -1){
        wheel_direction_1 = -1;
        wheel_direction_2 = -1;
        wheel_direction_3 = -1;
        wheel_direction_4 = -1;
        wheel_pwm_1 = rotating_speed * wheel_direction_1 ;
        wheel_pwm_2 = rotating_speed * wheel_direction_2 ;
        wheel_pwm_3 = rotating_speed * wheel_direction_3 ;
        wheel_pwm_4 = rotating_speed * wheel_direction_4 ;
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
        wheel_pwm_1 = speed_x * wheel_direction_1 ;
        wheel_pwm_2 = speed_x * wheel_direction_2 ;
        wheel_pwm_3 = speed_x * wheel_direction_3 ;
        wheel_pwm_4 = speed_x * wheel_direction_4 ;
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
        wheel_pwm_1 += speed_y * wheel_direction_1 ;
        wheel_pwm_2 += speed_y * wheel_direction_2 ;
        wheel_pwm_3 += speed_y * wheel_direction_3 ;
        wheel_pwm_4 += speed_y * wheel_direction_4 ;
        abs(wheel_pwm_1) > 255 ? wheel_pwm_1 = 255 : wheel_pwm_1 = wheel_pwm_1;
        abs(wheel_pwm_2) > 255 ? wheel_pwm_2 = 255 : wheel_pwm_2 = wheel_pwm_2;
        abs(wheel_pwm_3) > 255 ? wheel_pwm_3 = 255 : wheel_pwm_3 = wheel_pwm_3;
        abs(wheel_pwm_4) > 255 ? wheel_pwm_4 = 255 : wheel_pwm_4 = wheel_pwm_4;
    }
}