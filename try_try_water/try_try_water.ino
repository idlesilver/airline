#define readpin A0
#define pwm_out 1
#define in1 2
#define in2 3

int val;

void setup(){
    pinMode(readpin,INPUT);
    Serial.begin(9600);
}

void loop(){
    val = analogRead(readpin);
    Serial.println(val);
}