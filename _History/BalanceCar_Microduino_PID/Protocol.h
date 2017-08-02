#include <Arduino.h>

//BLE//////////////////////
#ifdef BLE_SoftSerial
#include <Microduino_Protocol_SoftSer.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(4, -1); // RX, TX (D5与电机冲突 屏蔽 只用RX)
Protocol bleProtocol(&mySerial, TYPE_NUM);  //软串口,校验数据类
#else
#include <Microduino_Protocol_HardSer.h>
Protocol bleProtocol(&BLE_HardSerial, TYPE_NUM);  //软串口,校验数据类
#endif

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
    switch (bleProtocol.parse(_channel, MODE_WHILE)) {
      case P_NONE:  //DATA NONE
        break;
      case P_FINE:  //DATA OK
        return true;
        break;
      case P_ERROR: //DATA ERROR
#ifdef BLE_SoftSerial
        mySerial.stopListening();
        mySerial.listen();
#endif
        break;
      case P_TIMEOUT: //DATA TIMEOUT
#ifdef BLE_SoftSerial
        mySerial.stopListening();
        mySerial.listen();
#endif
        break;
    }
  }

  return false;
}
