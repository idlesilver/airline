void setup(){
    Serial.begin(115200);
    Serial.print("not map:");
    Serial.println(100/255.0*2.0*4-2.0*2);//要有255.0
    Serial.print("map:");
    Serial.println(map(150,0,255,-2.0*2,2.0*2));
}

void loop(){

}