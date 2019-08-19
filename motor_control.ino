#define WHEEL_PWM_1 1
#define WHEEL_IN1_1 2
#define WHEEL_IN2_1 3
#define WHEEL_PWM_2 4
#define WHEEL_IN1_2 5
#define WHEEL_IN2_2 6
#define WHEEL_PWM_3 7
#define WHEEL_IN1_3 8
#define WHEEL_IN2_3 9
#define WHEEL_PWM_4 10
#define WHEEL_IN1_4 11
#define WHEEL_IN2_4 12

volatile int wheel_pwm_1 = 0;
volatile int wheel_pwm_2 = 0;
volatile int wheel_pwm_3 = 0;
volatile int wheel_pwm_4 = 0;



void setup(){}
void loop(){}



void motor_control(){
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