//BLE//////////////////////
//#include <Microduino_Protocol.h>
//ProtocolSer bleProtocol(&BLE_HardSerial, 26);  //软串口,校验数据类
#include <Microduino_Protocol.h>
ProtocolSer bleProtocol(&BLE_HardSerial, 16);  //软串口,校验数据类
//0xC8

//nRF//////////////////////
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
RF24 radio(9, 10);
RF24Network network(radio);
#define this_node  1  //设置本机ID
#define other_node 0
struct send_a { //发送
  uint32_t node_ms;   //节点运行时间
};
struct receive_a { //接收
  uint32_t ms;
  uint16_t rf_CH[CHANNEL_NUM];
};

//Mode//////////////////////
enum _Mode {
  NRF,
  BLE
};

uint32_t protocolTime = millis();

bool protocolSetup() {
  SPI.begin();    //初始化SPI总线
  radio.begin();
  if (radio.isPVariant()) {
    network.begin(NRF_CHANNEL, this_node);
    return NRF;
  }
  else  {
    bleProtocol.begin(BLE_SPEED);
    return BLE;
  }
}

bool protocolRead(uint16_t *_channel, bool _mode) {
  if (_mode == NRF) {
    network.update();
    while ( network.available() ) {
      RF24NetworkHeader header;
      receive_a rec;
      network.read(header, &rec, sizeof(rec));
      for (int a = 0; a < CHANNEL_NUM; a++) {
        _channel[a] = rec.rf_CH[a];
      }

      {
        send_a sen = { millis() };  //把这些数据发送出去，对应前面的发送数组
        RF24NetworkHeader header(other_node);
        if (network.write(header, &sen, sizeof(sen))) {
          return true;
        }
      }
    }
  }
  else  {
    if (bleProtocol.available()) {
      uint8_t recCmd;
      bleProtocol.readWords(&recCmd, _channel, CHANNEL_NUM);
      //      Serial.print("recCmd: ");
      //      Serial.print(recCmd);
      //      Serial.print("  Data:");
      for (uint8_t i = 0; i < 2; i++) {
        Serial.print(" ");
        Serial.print(_channel[i]);
      }
      Serial.println();
      return true;
    }
  }


  return false;
}
