#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class gyro
{ 
  Adafruit_MPU6050 mp;
  float pitch=0.0;
  float roll= 0.0;
  float yaw=0.0;
  unsigned long lastTime=0;

  public:
  void initialization(void)
  {
    Serial.begin(115200);
    unsigned long startTime = millis(); //unsigned as time cant be negative

    while (!Serial && (millis() - startTime < 3000)) {
      delay(10);
    }
    Serial.println("MPU6050 chip test.");

    if(!mp.begin())
    {
      Serial.println("Failed to find chip.");

      while(true)
      {
        delay(10);
      }
    }
    Serial.println("MPU6050 found.");

    mp.setAccelerometerRange(MPU6050_RANGE_8_G); //measurement range for the accl.

    mp.setGyroRange(MPU6050_RANGE_500_DEG); //measurement range for the gyro
    
    mp.setFilterBandwidth(MPU6050_BAND_5_HZ); //measurement range for the bandwith
    
    lastTime=millis();

    Serial.println("Setup done");
  }

  void values()
  {
    sensors_event_t a,g,t;
    mp.getEvent(&a,&g,&t);

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // time in seconds
    lastTime = currentTime;

    float ap= atan2(a.acceleration.y, a.acceleration.z)*180.0/M_PI;
    float ar= atan2(-a.acceleration.y, sqrt(a.acceleration.y*a.acceleration.y+a.acceleration.z*a.acceleration.z ))*180.0/M_PI; //180/pi for radians to degree

    float gp= g.gyro.x* 180/M_PI;
    float gr= g.gyro.y* 180/M_PI;
    float gy= g.gyro.z* 180/M_PI;

    pitch= 0.96*(pitch+gp*dt)+0.04*ap;
    roll= 0.96*(roll+gr*dt)+0.04*ar;

    yaw += gy * dt;

    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print("| Roll: ");
    Serial.print(roll);
    Serial.print("| Yaw: ");
    Serial.println(yaw);
  }
};

gyro g;
unsigned long previousMillis = 0;
const long interval = 500;
void setup()
{
  g.initialization();
}
void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis; //using millis instead of delay here so that the microcontroller can still work
    g.values();
  }
}