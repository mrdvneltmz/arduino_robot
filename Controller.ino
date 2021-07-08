#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>  // ce 9 csn 10 sck 13 mosi 11 miso 12
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  //Sda pin a4 Scl pin a5

#define joyX1 A0
#define joyX2 A2
#define joyY1 A1
#define joyY2 A3
#define modButon 7
#define lcdButon 6
#define yesilLed 2
#define sariLed 3
#define kirmiziLed 4
const byte adres[][6]={"00001", "00002"};  //Alıcı kanalı ve verici kanalı için 2 farklı adres

RF24 radio(9,10);
boolean durum=1;
boolean lcdMod=1;

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

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


LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

void carpisma(){   // Olası çarpışma durumları için uyarıları gösteren fonksiyon. (Platform üzerindeki fonksiyon gerekli aksiyonları alacak.)
  if(data.yakinlikOn==1){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.println("Yakinlik uyarisi");
      lcd.println(data.mesafeOn);
      lcd.println("Geri gidiniz");
     }else if(data.yakinlikArka==1){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.println("Yakinlik uyarisi");
      lcd.println(data.mesafeArka);
      lcd.println("İleri gidiniz");
     }if(data.yakinlikSol==1 && data.yakinlikSag==1){
      lcd.clear();
      lcd.println("Buradan gecemez");
     }if(data.yakinlikSol==1 && data.yakinlikSag==0){
      lcd.clear();
      lcd.println("Saga ilerleyin");
     }if(data.yakinlikSol==0 && data.yakinlikSag==1){
      lcd.clear();
      lcd.println("Sola ilerleyin");
     }
}



void setup() {
  pinMode(modButon,INPUT_PULLUP);
  pinMode(yesilLed,OUTPUT);
  pinMode(kirmiziLed,OUTPUT);
  pinMode(sariLed,OUTPUT);
  pinMode(lcdButon,INPUT_PULLUP);
 
  digitalWrite(sariLed,HIGH);
  digitalWrite(kirmiziLed,LOW);
  digitalWrite(yesilLed,LOW);

  durum=0;
  lcdMod=0;
  radio.begin(); 
  radio.openWritingPipe(adres[1]);    // Veri yazma kanalı
  radio.openReadingPipe(1,adres[0]);  // Veri okuma kanalı
  radio.setAutoAck(false);
  
  radio.setPALevel(RF24_PA_MIN);      //Sinyal güç ayarı. Min - Max // Low - High olarak ayarlanabilir

  lcd.init(); // LCD ekranın hazırlanması. Cursorun başa alınması ve sistem başlatılıyor ekranı
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Sistem baslatiliyor.");
  delay(1000);
  lcd.clear();

  //Default kontrol değerlerinin atanması 
  data.jX1=127;
  data.jX2=127;
  data.jY1=127;
  data.jY2=127;
  data.mod=0;
}

void loop() {
  digitalWrite(sariLed,HIGH);
  delay(5);
  radio.stopListening();                    // Veri gönderme moduna geçişi sağlıyor. (Transmitter)
  if(durum==1){                              // Motorların kontrol edilmesi durumu
    digitalWrite(yesilLed,LOW);
    digitalWrite(kirmiziLed,HIGH);
  }else{                                    // Kolun kontrol edilmesi durumu
    digitalWrite(kirmiziLed,LOW);
    digitalWrite(yesilLed,HIGH);
  }
  
  data.jX1 = map(analogRead(joyX1), 0, 1023, 0, 255);   // Joysticklerden okunan değerlerin -90 / +90 aralığına çevrilmesi. Joystick aşağı
  data.jX2 = map(analogRead(joyX2), 0, 1023, 0, 255);   // itildiğinde geri, ileri itildiğinde ileri hareketi sağlamak için -+90 aralığına
  data.jY1 = map(analogRead(joyY1), 0, 1023, 0, 255);   // çevirdik.
  data.jY2 = map(analogRead(joyY2), 0, 1023, 0, 255);
  
  if(digitalRead(modButon)==0){  // Eğer tuşa basıldı ise mod değişimi. INPUT_PULLUP kullanıldığı için 0'a göre işlem yapılmalı. 
    durum=!durum;
    data.mod=durum;
  }
  radio.write(&data,sizeof(Sinyal)); //Veri paketinin gönderilmesi
  delay(5);

  radio.startListening();                   // Veri okuma moduna geçişi sağlıyor. (Receiver)
  while(!radio.available()){                // Veri gürültüsünü önlemek için sadece radio aktarımı varken dinliyor.
    radio.read(&data,sizeof(Sinyal));   // Gelen veri paketi okunuyor
    lastReceiveTime = millis();
  }
  currentTime = millis();
  if(currentTime - lastReceiveTime > 1000){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.println("!!BAGLANTI KOPTU!!");
    digitalWrite(sariLed,LOW);
  }
    if(digitalRead(lcdButon)==0){  //Eğer butona basıldı ise LCD mod değişimi
      lcdMod=!lcdMod;
   }
  if(lcdMod==1){               //LCD birincil modu
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(data.sicaklik);
    lcd.print("'C  %");
    lcd.println(data.nem);
    lcd.print("X:");
    lcd.print(map(data.egimX,255,0,-90,90));
    lcd.print("  Y:");
    lcd.print(map(data.egimY,0,255,-90,90));
    carpisma();               //Uyarı fonksiyonu
     
  }if(lcdMod==0){             // LCD ikincil modu
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("On ");
    lcd.print(data.mesafeOn);
    lcd.println(" cm");
    lcd.print("Arka ");
    lcd.print(data.mesafeArka);
    lcd.println(" cm");
    carpisma();               //Uyarı fonksiyonu
    }
    
  }
