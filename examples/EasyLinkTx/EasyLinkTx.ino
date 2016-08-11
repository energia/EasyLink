#include "EasyLink.h"

EasyLink_TxPacket txPacket;
volatile uint8_t transmitted;
uint32_t packet_number = 0;

EasyLink myLink;

void setup() {
  Serial.begin(115200);
  pinMode(GREEN_LED, OUTPUT);
  // begin defaults to EasyLink_Phy_50kbps2gfsk
  myLink.begin();
  
  txPacket.dstAddr[0] = 0xaa;
  txPacket.absTime = 0;

  //Tx "hello world\n"
  txPacket.len = 12;
  txPacket.payload[0] = 'h';
  txPacket.payload[1] = 'e';
  txPacket.payload[2] = 'l';
  txPacket.payload[3] = 'l';
  txPacket.payload[4] = 'o';
  txPacket.payload[5] = ' ';
  txPacket.payload[6] = 'w';
  txPacket.payload[7] = 'o';
  txPacket.payload[8] = 'r';
  txPacket.payload[9] = 'l';
  txPacket.payload[10] = 'd';
  txPacket.payload[11] = '\n';

}

void loop() {
  myLink.transmit(&txPacket, &txDone);
  if(transmitted == true) {
    flashLed(2);
    Serial.print("Transmitted: #");
    Serial.println(packet_number++);
  }
  delay(1000);
}

void flashLed(uint8_t numFlashes) {
  while(numFlashes > 0){
    digitalWrite(GREEN_LED, HIGH);
    delay(50);
    digitalWrite(GREEN_LED, LOW);
    delay(50);
    numFlashes--;
  }
}

void txDone(EasyLink_Status status) {
  transmitted = true;
}

