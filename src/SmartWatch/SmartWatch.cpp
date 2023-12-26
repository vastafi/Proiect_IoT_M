#include <Arduino.h>

#include "SmartWatch/SmartWatch.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

MAX30105 particleSensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred


uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

float beatsPerMinute;
int beatAvg;
byte pulseLED = 25; //Must be on PWM pin, choose a suitable pin
byte readLED = 26; //Choose a suitable pin

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;  // the number of the pushbutton pin


// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status
int iteration=0;
int iteration_grahpics_BMP=0;
int iteration_grahpics_SpO=0;

void setup() {
  Serial.begin(9600);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.display();   
}

void display_temperature( float temperature){
display.clearDisplay();
  display.setCursor(0,0);
  display.print("Temp:");
  display.print(temperature, 1);
  display.println("C");
 display.display();
 delay(500);
}

void display_heart_rate(float BMP){

display.clearDisplay();
  display.setCursor(0,0);
  display.print("BMP :");
  display.println(int(BMP), 1);
 display.display();
 delay(500);
  
}

void display_SpO2(int32_t O2){
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("SpO2:");
  display.print(O2, DEC);
  display.println("%");
   display.display();
 delay(500);
  
}

void loop() {

  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  Serial.print("Buton:");
  Serial.println(buttonState);
  Serial.print("Iteratia:");
  Serial.println(iteration);
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    iteration++;
      delay(500);
  }
  if(iteration==1){
     particleSensor.setup(0); //Configure sensor. Turn off LEDs
  //particleSensor.setup(); //Configure sensor. Use 25mA for LED drive

  particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
  float temperature = particleSensor.readTemperature();

  Serial.print("temperatureC=");
  Serial.print(temperature, 4);

    display_temperature(temperature);
  
  }else if(iteration==2){
    
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
    if(iteration_grahpics_BMP==0){
      display.clearDisplay();
      display.setCursor(0,0);
     display.print("BMP : ...");
     display.display();
     delay(500);
    }
    iteration_grahpics_BMP++;
   long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  if(beatsPerMinute>50) display_heart_rate(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);


  if (irValue < 50000){
    Serial.print(" No finger?");
  }
  
  Serial.println();
  
  }else if(iteration==3){
      byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
   particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
    if(iteration_grahpics_SpO==0){
       display.clearDisplay();
       display.setCursor(0,0);
       display.print("SpO2: ...");
       display.display();
       delay(500);
    }
    iteration_grahpics_SpO++;
    iteration_grahpics_BMP=0;
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
      buttonState = digitalRead(buttonPin);
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      buttonState = digitalRead(buttonPin);
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);
      if(spo2<100 && spo2!=-999){
        display_SpO2(spo2);
      }
      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
   
        if (buttonState == HIGH) {
      delay(700);
      break;
  }
    }
         if (buttonState == HIGH) {
    iteration++;
      delay(700);
      break;
  }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
    
  }else if(iteration==4){
    iteration_grahpics_SpO=0;
    iteration=1;
  }
  else if(iteration>4){
    iteration=4;
  }

}


