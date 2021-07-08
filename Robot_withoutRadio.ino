 #include <SPI.h>
 #include <Wire.h>
 #include <Servo.h>
 #include <DHT.h>
 #include <NewPing.h>
 #include <LiquidCrystal_I2C.h>

 LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

 

 Servo servo1;
 Servo servo2;
 Servo servo3;
 Servo servo4;

 
 
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
 #define modButon 33
 #define yesilLed 34
 #define kirmiziLed 35
 #define maviLed 36
 #define jX1 A9
 #define jY1 A10
 #define jX2 A11
 #define jY2 A12

 NewPing onHC(onHC_Trig, onHC_Echo, MAX_MENZIL);
 NewPing arkaHC(arkaHC_Trig, arkaHC_Echo, MAX_MENZIL);
 DHT dht(DHTpin, DHTTYPE);
 
 int nem = 0;
 float sicaklik = 0;
 int mesafeOn,yakinlikOn,mesafeArka,yakinlikArka,yakinlikSag,yakinlikSol;
 int durum = 0;
 float gaz;
void carpisma(){   // Olası çarpışma durumları için uyarıları gösteren fonksiyon. (Platform üzerindeki fonksiyon gerekli aksiyonları alacak.)
  if(yakinlikOn==1){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.println("Yakinlik uyarisi");
      lcd.println(mesafeOn);
      lcd.println("Geri gidiniz");
     }else if(yakinlikArka==1){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.println("Yakinlik uyarisi");
      lcd.println(mesafeArka);
      lcd.println("İleri gidiniz");
     }if(yakinlikSol==1 && yakinlikSag==1){
      lcd.clear();
      lcd.println("Buradan gecemez");
     }if(yakinlikSol==1 && yakinlikSag==0){
      lcd.clear();
      lcd.println("Saga ilerleyin");
     }if(yakinlikSol==0 && yakinlikSag==1){
      lcd.clear();
      lcd.println("Sola ilerleyin");
     }
}

void dhtSensor(){
  nem = dht.readHumidity();
  sicaklik = dht.readTemperature();
}

void onHC_Kontrol(){
  mesafeOn = onHC.ping_cm();
  if(mesafeOn<=5){
    yakinlikOn = 1;
  }else{
    yakinlikOn = 0;
  }
}

void arkaHC_Kontrol(){
  mesafeArka = arkaHC.ping_cm();
  if(mesafeArka<=5){
    yakinlikArka = 1;
  }else{
    yakinlikArka = 0;
  }
}

void irKontrol(){
  if(digitalRead(ir_Sol)==1){
    yakinlikSol=1;
  }else{
    yakinlikSol=0;
  }if(digitalRead(ir_Sag)==1){
    yakinlikSag=1;
  }else{
    yakinlikSag=0;
  }
}

void motorKontrol(){
     /* if(Y1==1){                      //Sol joystick senaryosu
        digitalWrite(MotorSol_Geri,LOW);
        digitalWrite(MotorSag_Geri,LOW);
        digitalWrite(MotorSol_Ileri,HIGH);
        digitalWrite(MotorSag_Ileri,HIGH);
      }else if(Y1==0){
        digitalWrite(MotorSol_Ileri,LOW);
        digitalWrite(MotorSag_Ileri,LOW);
        digitalWrite(MotorSol_Geri,HIGH);
        digitalWrite(MotorSag_Geri,HIGH);
      }else if(X1==1){  //Saga donus
        digitalWrite(MotorSol_Geri,LOW);
        digitalWrite(MotorSol_Ileri,HIGH);
        digitalWrite(MotorSag_Geri,LOW);
        digitalWrite(MotorSag_Ileri,LOW);      
      }else if(X1==0){  //Sola donus
        digitalWrite(MotorSol_Geri,LOW);
        digitalWrite(MotorSol_Ileri,LOW);
        digitalWrite(MotorSag_Geri,LOW);
        digitalWrite(MotorSag_Ileri,HIGH);      
      }
      */
      int X1 = map(analogRead(jX1), 0, 1023, -90, 90);
      int Y1 = map(analogRead(jY1), 0, 1023, -90, 90);
      int X2 = map(analogRead(jX2), 0, 1023, -90, 90);
      int Y2 = map(analogRead(jY2), 0, 1023, -90, 90);
        if(X2<-5){
          digitalWrite(MotorSol_Geri,HIGH);
          digitalWrite(MotorSag_Geri,HIGH);
          digitalWrite(MotorSol_Ileri,LOW);
          digitalWrite(MotorSag_Ileri,LOW);        
        }else if(X2>5){
          digitalWrite(MotorSol_Geri,LOW);
          digitalWrite(MotorSag_Geri,LOW);
          digitalWrite(MotorSol_Ileri,HIGH);
          digitalWrite(MotorSag_Ileri,HIGH);        
        }else if(Y1<-5){  //Sola donus
          digitalWrite(MotorSol_Geri,HIGH);
          digitalWrite(MotorSol_Ileri,LOW);
          digitalWrite(MotorSag_Geri,LOW);
          digitalWrite(MotorSag_Ileri,HIGH);        
        }else if(Y1>5){  //Saga donus 
          digitalWrite(MotorSol_Geri,LOW);
          digitalWrite(MotorSol_Ileri,HIGH);
          digitalWrite(MotorSag_Geri,HIGH);
          digitalWrite(MotorSag_Ileri,LOW);        
        }else if(X2<5 && X2>-5 && Y1<5 && Y1>-5){
          digitalWrite(MotorSol_Geri,LOW);
          digitalWrite(MotorSag_Geri,LOW);
          digitalWrite(MotorSol_Ileri,LOW);
          digitalWrite(MotorSag_Ileri,LOW); 
        }
      }
    
void sifirla(){
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
}
int a=90,b=90,c=90,d=90;

void servoKontrol(){
    
    int servoX1 = map(analogRead(jX1), 0, 1023, 0, 180);
    int servoY1 = map(analogRead(jY1), 0, 1023, 0, 180);
    int servoX2 = map(analogRead(jX2), 0, 1023, 0, 180);
    int servoY2 = map(analogRead(jY2), 0, 1023, 0, 180);
    Serial.println(servoX1);
    Serial.println(servoY1);
    Serial.println(servoX2);
    Serial.println(servoY2);
    if(servoX1>95){
      a=a+5;
      if(a<184){
        servo1.write(a);
        delay(50);
      }
    }else if(servoX1<85){
      a=a-5;
      if(a>0){
        servo1.write(a);
        delay(50);
    }
    }
    if(servoY1>95){
      b=b+5;
      if(b<184){
        servo2.write(b);
        delay(50);
      }
    }else if(servoY1<85){
      b=b-5;
      if(b>0){
        servo2.write(b);
        delay(50);
      }
    }
    if(servoX2>95){
      c=c+5;
      if(c<184){
        servo3.write(c);
        delay(50);
      }
    }else if(servoX2<85){
      c=c-5;
      if(c>0){
        servo3.write(c);
        delay(50);
      }
    }
    if(servoY2>95){
      d=d+5;
      if(d<184){
        servo4.write(d);
        delay(50);
      }
    }else if(servoY2<85){
      d=d-5;
      if(d>0){
        servo4.write(d);
        delay(50);
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
   gaz = ppm;
}

void setup() {
  Serial.begin(9600);
  pinMode(modButon,INPUT_PULLUP);
  pinMode(yesilLed,OUTPUT);
  pinMode(kirmiziLed,OUTPUT);
  pinMode(maviLed,OUTPUT);

  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  
  dht.begin();
  
  durum=0;

  lcd.init(); // LCD ekranın hazırlanması. Cursorun başa alınması ve sistem başlatılıyor ekranı
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Sistem baslatiliyor.");
  digitalWrite(yesilLed,HIGH);
  digitalWrite(kirmiziLed,HIGH);
  digitalWrite(maviLed,HIGH);
  delay(5000);
  digitalWrite(kirmiziLed,LOW);
  digitalWrite(maviLed,LOW);

}

void loop() {
  delay(5);
  dhtSensor();
  onHC_Kontrol();
  arkaHC_Kontrol();
  irKontrol();
  MQ4_Olcum();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(sicaklik);
  lcd.print("'C  Nem %");
  lcd.print(nem);
  lcd.setCursor(0,1);
  lcd.print("On->");
  lcd.print(mesafeOn);
  lcd.setCursor(0,2);
  lcd.print("Arka->");
  lcd.print(mesafeArka);
  lcd.setCursor(0,3);
  lcd.print("PPM->");
  lcd.print(gaz);

  
  delay(200);
  
  
  
  /*int okunan_jX1 = digitalRead(jX1);
  int okunan_jY1 = digitalRead(jY1);
  int okunan_jX2 = digitalRead(jX2);
  int okunan_jY2 = digitalRead(jY2);
  */
  
  if(digitalRead(modButon)==LOW){  // Eğer tuşa basıldı ise mod değişimi. INPUT_PULLUP kullanıldığı için 0'a göre işlem yapılmalı. 
    durum=!durum;
  }
  if(durum==1){
    motorKontrol();
  }else{
    
    servoKontrol();
  }
}
