#define servopin 9
#define pos_time 2000

void setup(){
    pinMode(servopin,OUTPUT);
    Serial.begin(9600);
}

void loop(){
    digitalWrite(servopin,HIGH);
    delayMicroseconds(pos_time);
    digitalWrite(servopin,LOW);
    delayMicroseconds(20000-pos_time);
    Serial.println("da");
