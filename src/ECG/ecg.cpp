#include <Arduino.h>
#include "ECG/ecg.h"

void freqDetec() {
  if (countData==MAX_BUFFER) {
   if (prevData[roundrobin] < avgData*1.5 && newData >= avgData*1.5){ // increasing and crossing last midpoint
    period = millis()-millistimer;//get period from current timer value
    millistimer = millis();//reset timer
    maxData = 0;
   }
  }
 roundrobin++;
 if (roundrobin >= MAX_BUFFER) {
    roundrobin=0;
 }
 if (countData<MAX_BUFFER) {
    countData++;
    sumData+=newData;
 } else {
    sumData+=newData-prevData[roundrobin];
 }
 avgData = sumData/countData;
 if (newData>maxData) {
  maxData = newData;
 }

#ifdef PLOTT_DATA
  Serial.print(newData);
 Serial.print("\t");
 Serial.print(avgData);
 Serial.print("\t");
 Serial.print(avgData*1.5);
 Serial.print("\t");
 Serial.print(maxData);
 Serial.print("\t");
 Serial.println(beatspermin);
#endif
 prevData[roundrobin] = newData;//store previous value
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  newData = analogRead(34);
  freqDetec();
  if (period!=lastperiod) {
     frequency = 1000/(double)period;//timer rate/period
     if (frequency*60 > 20 && frequency*60 < 200) { // supress unrealistic Data
      beatspermin=frequency*60;
#ifndef PLOTT_DATA
        Serial.print(frequency);
        Serial.print(" hz");
        Serial.print(" ");
        Serial.print(beatspermin);
        Serial.println(" bpm");
#endif
        lastperiod=period;
     }
  }
  delay(5);
}