#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);  // 新建一个mpu6050实例

long timer = 0;  // 存储每次迭代开始时的时间
int angle;  // 相对于初始值旋转的角度，逆时针为正，顺时针为负
int init_angle;  // 角度初始值

void setup() {
    Serial.begin(9600);
    Wire.begin();             // 开启 I2C 总线
    mpu6050.begin();          // 开启mpu6050
    mpu6050.calcGyroOffsets(true);    // 计算初始位置
}
void loop(){
    get_pitch_angle();
}
inline void get_pitch_angle() {
    mpu6050.update();              // 更新当前位置
    if (millis() - timer > 500) {         // 每500ms更新一次当前位置
        Serial.print("The angle is: ");Serial.println(mpu6050.getGyroAngleY());
        timer = millis();
    }
}