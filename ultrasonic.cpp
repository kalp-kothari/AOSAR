#include <Arduino.h>
class ultrasonic:
{
private:
uint8_t trig, echo; //initializing pins

public:
ultrasonic(uint8_t trig, uint8_t echo) : t(trig), e(echo) //made constructor for easy access to the priv variables
    void initialize(uint8_t p, uint8_t io)
{
    //setup 
    pinMode(t, OUTPUT);
    pinMode(e, INPUT);
}
long read()
{
    //making trig send out signals
    digitalWrite(t,LOW);
    delayMicroseconds(2);
    digitalWrite(t,HIGH);
    delayMicroseconds(10);
    digitalWrite(t,LOW);

    //calculating the duration and distance
    long duration= pulseIn(e, HIGH);
    long distance= (0.034 * duration)/2;

    return distance;
}
};

Ultrasonic u(5,18);
void setup()
{
    Serial.begin(115200);
    u.initialize();
}
void loop()
{
    long read();
}












