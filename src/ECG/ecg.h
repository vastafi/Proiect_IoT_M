#define PLOTT_DATA 
#define MAX_BUFFER 100

uint32_t prevData[MAX_BUFFER];
uint32_t sumData=0;
uint32_t maxData=0;
uint32_t avgData=0;
uint32_t roundrobin=0;
uint32_t countData=0;
uint32_t period=0;
uint32_t lastperiod=0;
uint32_t millistimer=millis();
double frequency;
double beatspermin=0;
uint32_t newData;
