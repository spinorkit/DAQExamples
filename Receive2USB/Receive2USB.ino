#define BYTES_IN_BUFFER 62

uint8_t buf[BYTES_IN_BUFFER];
uint8_t rxBuf[BYTES_IN_BUFFER];

void setup() 
{
  for(int i(0);i<BYTES_IN_BUFFER;++i)
    {
    buf[i]='0'+i;
    rxBuf[i] = 0;
    }

  SerialUSB.begin(1e6);            //Baud rate is irrelevant
  while(!SerialUSB);
//  Serial.begin(9600);
//  while(!Serial);

}

enum State
{
kIdle,
kSampling,  
};

State gState = kIdle;

void loop() 
{
//uint8_t a = SerialUSB.readBytes(buf,BYTES_IN_BUFFER);
//if (a!=0) 
int rxAvail = SerialUSB.available();
if(rxAvail)
    {
    SerialUSB.readBytes(rxBuf,rxAvail);
    //SerialUSB.readString(); //reads until '\0' or timeout
    auto cmd = rxBuf[0];
    switch (cmd)
      {
      case 'b':
         gState = kSampling;
         break;
      case 's':
         gState = kIdle;
         break;
      default:
         break;
      }
    }


if(gState == kSampling)
   {
   SerialUSB.write(buf,BYTES_IN_BUFFER);
   delay(100);
   }
//  Serial.write(buf,BYTES_IN_BUFFER);
//  Serial.println();
//  Serial.println(a);

//  delay(800);
}
