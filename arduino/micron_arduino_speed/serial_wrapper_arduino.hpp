/**
*    @author : koseng (Lintang)
*    @brief : serial wrapper
*/
#pragma once

#include <Arduino.h>
#include "circular_buffer.hpp"

namespace micron{

class SerialWrapper{
public:
  SerialWrapper(Stream* _stream);
  ~SerialWrapper();

  inline void transmit(unsigned char _data);
  template <typename Type>
  inline void sendData(Type _data);
  inline String getData();
  void receiveRoutine();
  
private:
  CircularBuffer<String> rpacket_buffer_;
  
  Stream* stream_;
};

SerialWrapper::SerialWrapper(Stream* _stream)
  : rpacket_buffer_(1)
  , stream_(_stream){
}

SerialWrapper::~SerialWrapper(){
  stream_ = NULL;
}

void SerialWrapper::transmit(unsigned char _data){
  stream_->write(_data);
}

template <typename Type>
void SerialWrapper::sendData(Type _data){
  String str_data(_data);
  for(size_t i(0); i < str_data.length(); i++){
    transmit( str_data.charAt(i) );
  }
  transmit('\n');
  stream_->flush();
}

String SerialWrapper::getData(){
  return rpacket_buffer_.at(0);
}

void SerialWrapper::receiveRoutine(){
  static String packet;
  while(stream_->available()){
    char c = stream_->read();
    if(c == '\n'){
      rpacket_buffer_.push(packet);
      packet = "";
    }else{
      packet += c;
    }    
  }
//  delay(1);
}

}
