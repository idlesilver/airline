#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);  // 新建一个mpu6050实例

long timer = 0;  // 存储每次迭代开始时的时间
int angle;  // 相对于初始值旋转的角度，逆时针为正，顺时针为负
int init_angle;  // 角度初始值

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);  
}

void loop() {
    mpu6050.update();

    if (millis() - timer > 1000) {
        angle = mpu6050.getGyroAngleY() - init_angle;
        Serial.print("The angle is: ");Serial.println(mpu6050.getGyroAngleY());
        timer = millis();
    }
}