 #include <SPI.h>
 #include <nRF24L01.h>
 #include <RF24.h>  // ce 7 csn 8 sck 13 mosi 11 miso 12
 #include <Wire.h>
 #include <Servo.h>
 #include <DHT.h>
 #include <NewPing.h>
 
 #define DHTpin 32
 #define DHTTYPE DHT11
 #define MotorSol_Ileri 28
 #define MotorSol_Geri 29
 #define MotorSag_Ileri 30
 #define MotorSag_Geri 31
 #define MQ4pin A8
 #define sensorMin = 0; // sensor maksimum
 #define sensorMax = 1024; // sensor minimum
 #define ir_Sol 22
 #define ir_Sag 23
 #define onHC_Trig 24
 #define onHC_Echo 25
 #define arkaHC_Trig 27
 #define arkaHC_Echo 26
 #define MAX_MENZIL 200

 NewPing onHC(onHC_Trig, onHC_Echo, MAX_MENZIL);
 NewPing arkaHC(arkaHC_Trig, arkaHC_Echo, MAX_MENZIL);
 
 Servo servo1;
 Servo servo2;
 Servo servo3;
 Servo servo4;

 DHT dht(DHTpin, DHTTYPE);
 
 float AccX, AccY, AccZ;
 float GyroX, GyroY, GyroZ;
 float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
 float angleX, angleY;
 float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY;
 float elapsedTime, currentTime, previousTime;
 int c = 0;
 const int MPU = 0x68; 

 unsigned long lastReceiveTime = 0;
 unsigned long currentTime1 = 0;
 
 const byte adres[][6]={"00001", "00002"};
 RF24 radio(7,8); 


struct Sinyal{  
  byte sicaklik;
  byte nem;
  byte egimX;
  byte egimY;
  byte gaz;
  byte yakinlikOn;
  byte yakinlikArka;
  byte yakinlikSol;
  byte yakinlikSag;
  byte mesafeOn;
  byte mesafeArka;
  byte jX1;
  byte jX2;
  byte jY1;
  byte jY2;
  byte mod;
}data;

void sifirla(){
  data.sicaklik = 0;
  data.nem = 0;
  data.egimX = 127;
  data.egimY = 127;
  data.gaz = 0;
  data.yakinlikOn = 1;
  data.yakinlikArka = 1;
  data.yakinlikSol = 1;
  data.yakinlikSag = 1;
  data.mesafeOn = 5;
  data.mesafeArka =5;
}

void dhtSensor(){
  data.nem = dht.readHumidity();
  data.sicaklik = dht.readTemperature();
}

void onHC_Kontrol(){
  data.mesafeOn = onHC.ping_cm();
  if(data.mesafeOn<=5){
    data.yakinlikOn = 1;
  }else{
    data.yakinlikOn = 0;
  }
}

void arkaHC_Kontrol(){
  data.mesafeArka = arkaHC.ping_cm();
  if(data.mesafeArka<=5){
    data.yakinlikArka = 1;
  }else{
    data.yakinlikArka = 0;
  }
}

void irKontrol(){
  if(digitalRead(ir_Sol)==1){
    data.yakinlikSol=1;
  }else{
    data.yakinlikSol=0;
  }if(digitalRead(ir_Sag)==1){
    data.yakinlikSag=1;
  }else{
    data.yakinlikSag=0;
  }
}

void motorKontrol(byte X1, byte Y1, byte X2, byte Y2){
    if(X2==127 && Y2==127){
      if(Y1>127 && data.yakinlikOn!=1){                      //Sol joystick senaryosu
        digitalWrite(MotorSol_Geri,LOW);
        digitalWrite(MotorSag_Geri,LOW);
        digitalWrite(MotorSol_Ileri,HIGH);
        digitalWrite(MotorSag_Ileri,HIGH);
      }else if(Y1<127 && data.yakinlikArka!=1){
        digitalWrite(MotorSol_Ileri,LOW);
        digitalWrite(MotorSag_Ileri,LOW);
        digitalWrite(MotorSol_Geri,HIGH);
        digitalWrite(MotorSag_Geri,HIGH);
      }else if(X1>127 && data.yakinlikSag!=1){  //Saga donus
        digitalWrite(MotorSol_Geri,LOW);
        digitalWrite(MotorSol_Ileri,HIGH);
        digitalWrite(MotorSag_Geri,LOW);
        digitalWrite(MotorSag_Ileri,LOW);      
      }else if(X1<127 && data.yakinlikSol!=1){  //Sola donus
        digitalWrite(MotorSol_Geri,LOW);
        digitalWrite(MotorSol_Ileri,LOW);
        digitalWrite(MotorSag_Geri,LOW);
        digitalWrite(MotorSag_Ileri,HIGH);      
      }
      if(X1==127 && Y1==127){
        if(Y2>127 && data.yakinlikArka !=1 ){
          digitalWrite(MotorSol_Geri,HIGH);
          digitalWrite(MotorSag_Geri,HIGH);
          digitalWrite(MotorSol_Ileri,LOW);
          digitalWrite(MotorSag_Ileri,LOW);        
        }else if(Y2<127&& data.yakinlikOn !=1){
          digitalWrite(MotorSol_Geri,LOW);
          digitalWrite(MotorSag_Geri,LOW);
          digitalWrite(MotorSol_Ileri,HIGH);
          digitalWrite(MotorSag_Ileri,HIGH);        
        }else if(X2>127 && data.yakinlikSol!=1){  //Sola donus
          digitalWrite(MotorSol_Geri,LOW);
          digitalWrite(MotorSol_Ileri,LOW);
          digitalWrite(MotorSag_Geri,LOW);
          digitalWrite(MotorSag_Ileri,HIGH);        
        }else if(X2<127 && data.yakinlikSag!=1){  //Saga donus
          digitalWrite(MotorSol_Geri,LOW);
          digitalWrite(MotorSol_Ileri,HIGH);
          digitalWrite(MotorSag_Geri,LOW);
          digitalWrite(MotorSag_Ileri,LOW);        
        }
      }
    }
}

void servoKontrol(byte servoX1,byte servoY1,byte servoX2,byte servoY2){
  if(servoX1>90){
      for(int i=90;i=servoX1;i++){
        servo1.write(i);
        delay(5);
      }
    }else if(servoX1<90){
      for(int i=90;i=servoX1;i--){
        servo1.write(i);
        delay(5);
      }
    }if(servoY1>90){
      for(int i=90;i=servoY1;i++){
        servo2.write(i);
        delay(5);
      }
    }else if(servoY1<90){
      for(int i=90;i=servoY1;i--){
        servo2.write(i);
        delay(5);
      }
    }if(servoX2>90){
      for(int i=90;i=servoX2;i++){
        servo3.write(i);
        delay(5);
      }
    }else if(servoX2<90){
      for(int i=90;i=servoX2;i--){
        servo3.write(i);
        delay(5);
      }
    }if(servoY2>90){
      for(int i=90;i=servoY2;i++){
        servo4.write(i);
        delay(5);
      }
    }else if(servoY2<90){
      for(int i=90;i=servoY2;i--){
        servo4.write(i);
        delay(5);
      }
    }
  
}

void MQ4_Olcum(){
  float m = -0.318;
  float b = 1.133; 
  float R0 = 11.820; 
  float sensorVoltaj; 
  float sensorDirenc_gas; 
  float oran; //
  float MQ4_Deger = analogRead(MQ4pin); 
  sensorVoltaj = MQ4_Deger * (5.0 / 1023.0); 
  sensorDirenc_gas = ((5.0 * 10.0) / sensorVoltaj) - 10.0; //RS = [(Vin x RL) / Vout] - RL
  oran = sensorDirenc_gas / R0;  

  double ppm_log = (log10(oran) - b) / m; 
  double ppm = pow(10, ppm_log); 
   ppm = ppm / 100000;
   data.gaz = ppm;
}

void initialize_MPU6050() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Configure Accelerometer
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);
}

void read_IMU(){
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-8g, we need to divide the raw values by 4096, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value
  // Calculating angle values using
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 1.15; // AccErrorX ~(-1.15) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - 0.52; // AccErrorX ~(0.5)
  // === Read gyro data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;   // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8; // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroX = GyroX + 1.85; //// GyroErrorX ~(-1.85)
  GyroY = GyroY - 0.15; // GyroErrorY ~(0.15)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = GyroX * elapsedTime;
  gyroAngleY = GyroY * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  angleX = 0.98 * (angleX + gyroAngleX) + 0.02 * accAngleX;
  angleY = 0.98 * (angleY + gyroAngleY) + 0.02 * accAngleY;
  // Map the angle values from -90deg to +90 deg into values from 0 to 255, like the values we are getting from the Joystick
  data.egimX = map(angleX, -90, +90, 255, 0);
  data.egimY = map(angleY, -90, +90, 0, 255);
}


void setup() {
  void sifirla();
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);

  pinMode(MotorSol_Ileri,OUTPUT);
  pinMode(MotorSol_Geri,OUTPUT);
  pinMode(MotorSag_Ileri,OUTPUT);
  pinMode(MotorSag_Geri,OUTPUT);
  pinMode(ir_Sol,INPUT);
  pinMode(ir_Sag,INPUT);
 
  
  digitalWrite(MotorSol_Geri,LOW);
  digitalWrite(MotorSag_Geri,LOW);
  digitalWrite(MotorSol_Ileri,LOW);
  digitalWrite(MotorSag_Ileri,LOW);
      
  dht.begin();
  initialize_MPU6050();
  delay(1000);
  
  radio.begin();                            //Starting the radio communication
  radio.openWritingPipe(adres[0]);      //Setting the address at which we will send the data
  radio.openReadingPipe(1, adres[1]);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN);            //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
}
  


void loop() {
  delay(5);
  dhtSensor();
  onHC_Kontrol();
  arkaHC_Kontrol();
  irKontrol();
  MQ4_Olcum();
  read_IMU();
  
  radio.startListening(); 
  while(!radio.available()){                         // Veri gürültüsünü önlemek için sadece radio aktarımı varken dinliyor.
    radio.read(&data,sizeof(Sinyal));     // Gelen veri paketi okunuyor
    lastReceiveTime = millis();                      // At this moment we have received the data
  }
  currentTime1 = millis();
  if(currentTime1 - lastReceiveTime > 1000){
    sifirla(); 
  }
  if(data.mod==0){
     motorKontrol(data.jX1,data.jY1,data.jX2,data.jY2);
  }else if(data.mod==1){
     servoKontrol(data.jX1,data.jY1,data.jX2,data.jY2);
    }
  radio.stopListening();                    // Veri gönderme moduna geçişi sağlıyor. (Transmitter)
  radio.write(&data,sizeof(Sinyal)); //Veri paketinin gönderilmesi
  delay(5);
}



