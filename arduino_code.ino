#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16,2);
int buz = 5;        //buzzer connected to pin 7
int fan = 6;         // fan connected to pin 6
int greenled = 8;   //green led connected to pin 8
int redled = 9;     //red led connected to pin 9

SoftwareSerial Ser(2, 3); // RX, TX for Nodemcu-ESP8266
int MQ135_sensor= A0;    //Sensor pin 
float m = -0.353;       //Slope 
float c = 0.711;        //Y-Intercept 
float R0 = 23.30;       //Sensor Resistance in fresh air from previous code 21.30
int MQ7_sensor = A1; //Sensor pin 
float m1 = -0.67; //Slope 
float c1 = 1.34; //Y-Intercept 
float R01 = 5.80; //Sensor Resistance 4.80
void setup() {
  Serial.begin(9600);     // PC to Arduino Serial Monitor
  Ser.begin(115200);      // Arduino to ESP01 Communication or 115200

  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.print(" Air  Pollution ");
  lcd.setCursor(0,1);
  lcd.print(" Monitor System ");
  delay(4000);
  lcd.clear();
  
  
  pinMode(buz,OUTPUT);         
  pinMode(greenled,OUTPUT);    
  pinMode(redled, OUTPUT);      
  pinMode(fan, OUTPUT);
  pinMode(MQ135_sensor, INPUT);
  pinMode(MQ7_sensor,INPUT);

  unsigned char check_connection=0;
  unsigned char times_check=0;
  Serial.println("Connecting to Wifi");
  lcd.print("Connect to Wifi.");
  delay(1000);
  lcd.clear();
  
  while(check_connection==0)
  {
     if(Ser.find("\nWIFI CONNECTED.")==1 )
     {
        Serial.println("WIFI CONNECTED.");
        lcd.print("WIFI CONNECTED.");
        delay(2000);
        lcd.clear();   
        break;
     }  
    times_check++;
    if(times_check>3) 
     {
        times_check=0;
        Serial.println("Trying to Reconnect..");
        lcd.setCursor(0,0);
        lcd.print("    Trying to   ");
        lcd.setCursor(0,1);
        lcd.print("  Reconnect...  ");
        delay(2000);
        lcd.clear();
      }
  }
}

void loop() { 
  float sensor_volt;          //Define variable for sensor voltage 
  float RS_gas;               //Define variable for sensor resistance  
  float ratio;                //Define variable for ratio
  float sensorValue = analogRead(MQ135_sensor);   //Read analog values of sensor  
  sensor_volt = sensorValue*(5.0/1024.0);       //Convert analog values to voltage 
  RS_gas = ((5.0*10.0)/sensor_volt)-10.0;       //Get value of RS in a gas
  ratio = RS_gas/R0;                            // Get ratio RS_gas/RS_air
  double ppm_log = (log10(ratio)-c)/m;          //Get ppm value in linear scale according to the the ratio value  
  double ppm = pow(10, ppm_log);                //Convert ppm value to log scale 
  
  Serial.print("Our Air Qualit PPM = ");
  Serial.println(ppm);
  Serial.println(ratio);
  
    
  lcd.setCursor(0,0);             // set cursor of lcd to 1st row and 1st column
  lcd.print("CO2: ");             // print message on lcd
  lcd.print(ppm);                 // print value of MQ135
  
  float sensor_volt1;             //Define variable for sensor voltage 
  float RS_gas1;                  //Define variable for sensor resistance  
  float ratio1;                   //Define variable for ratio
  float sensorValue1 = analogRead(MQ7_sensor);   //Read analog values of sensor  
  sensor_volt1 = sensorValue1*(5.0/1024.0);     //Convert analog values to voltage 
  RS_gas1 = ((5.0*10.0)/sensor_volt1)-10.0;     //Get value of RS in a gas
  ratio1 = RS_gas1/R01;                         // Get ratio RS_gas/RS_air
  double ppm_log1 = (log10(ratio1)-c1)/m1;      //Get ppm value in linear scale according to the the ratio value  
  double ppm1 = pow(10, ppm_log1);              //Convert ppm value to log scale 
  
  Serial.print("CO PPM = ");
  Serial.println(ppm1);

  Serial.println(analogRead(MQ135_sensor));
  Serial.println(analogRead(MQ7_sensor));
  
  lcd.setCursor(0,1);             // set cursor of lcd to 1st row and 1st column
  lcd.print("CO PPM = ");         // print message on lcd
  lcd.print(ppm1);                // print value of MQ7
  delay(5000);

    Ser.print('<'); // Starting char
    Ser.print(ppm); // float data
    Ser.print(',');
    Ser.print(ppm1); // float data
    Ser.println('>'); // Ending char
    lcd.clear();
    lcd.print("Upload to Cloud.");

    if (ppm >= 10 || ppm1 >= 10) {
      digitalWrite(greenled, LOW);
      digitalWrite(buz, HIGH);
      digitalWrite(redled, HIGH);
      digitalWrite(fan, HIGH);
      lcd.setCursor(0,1);
      lcd.print("Polluted Air");
      Serial.println("Alert!!!");
      delay(2000); // wait 2000ms
      lcd.clear();
    }
  
    else {
      digitalWrite(greenled, HIGH);
      digitalWrite(redled, LOW);
      digitalWrite(buz, LOW);
      digitalWrite(fan, LOW);
      lcd.setCursor(0,1);
      lcd.print(" Normal Air ");
      Serial.println("Normal");
      delay(2000); // wait 500ms
      lcd.clear();
    }
}
