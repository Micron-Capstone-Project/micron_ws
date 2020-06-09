#include <Wire.h>
#include <Adafruit_INA219.h>
#include <ArduinoJson.h>
#include "TimerOne.h"
#include "serial_wrapper_arduino.hpp"
#include "discrete_system.hpp"

//-- Driver pin
const static int PWM_R = 5;
const static int PWM_L = 6;
const static int EN_R = 4;
const static int EN_L = 7;
//-- Encoder pin
const static int OUT_A = 2;
const static int OUT_B = 8;
//-- Encoder stuff
volatile int counter(0);
volatile int last_counter(0);
const static double ENCODER_CONST = 1.0;//.7;
const static int PPR = 720;
const static double P2DEG = ENCODER_CONST*360./(double)PPR;
const double MAX_VOLTAGE = 12.0;
const double TO_PWM = 255./MAX_VOLTAGE;

//-- Speed sensor stuff
const static int SPEED_SENSOR_PIN = 3;
const static int NUM_LATTICE = 20;
volatile int ss_counter(0);
volatile int ss_last_counter(0);
volatile int ss_state(LOW);
volatile int ss_last_state(LOW);
volatile unsigned long start_time(0);
volatile unsigned long end_time(0);
double vel(.0);

//-- Current sensor
Adafruit_INA219 current_sensor;
double current(.0);

//-- Serial wrapper
micron::SerialWrapper* sw;
const int capacity = JSON_OBJECT_SIZE(10);
enum{
  Position,
  Velocity,
  Current
};

//-- Control stuff
typedef micron::DiscreteSystem<2,1,1> Controller;
//-- Gamma - 2.7
//--  Weight1 - 0.35
//-- Weight2 - 1.0
//-- jacl
Controller::state_matrix_t Ak = {
   0.018033, 0.737486,
   -0.001936, -0.079160 
};
Controller::input_matrix_t Bk = {
  -0.114770, 0.012322 
};
Controller::output_matrix_t Ck = {
  -0.046490, -1.901509
};
Controller::feedforward_matrix_t Dk = {
  0.295919
};
Controller::output_t in;
Controller::input_t out;
Controller controller(Ak,Bk,Ck,Dk);
const uint64_t SAMPLING_TIME(100); //microseconds
double in_voltage(.0);
auto ref(.0);
auto error(.0);
auto rpwm(.0);
auto lpwm(.0);
const auto TO_RAD(PI/180.);

void timerIsr(){
  Timer1.detachInterrupt();
  Timer1.attachInterrupt(timerIsr);
}

void pciSetup(int pin){
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}

const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
static unsigned char New, Old;

ISR(PCINT0_vect){
  Old = New << 2;
  New = (PINB & 1 )+ ((PIND & 4) >> 1);
  counter += QEM [Old + New];
}

void encoderInt() {
  Old = New << 2;
  New = (PINB & 1 )+ ((PIND & 4) >> 1);
  counter += QEM [Old + New];

}

void speedSensorISR(){
  static volatile unsigned long debounce(0);
  if(micros()-debounce > 550 && digitalRead(SPEED_SENSOR_PIN)){
    debounce = micros();
    ++ss_counter; 
  }  
}

void setup() {
  //-- Initialize serial
  Serial.begin(1000000);
  sw = new micron::SerialWrapper(&Serial);  
  //-- Driver setup
  pinMode(EN_R, OUTPUT);
  pinMode(EN_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  
  //-- Encoder setup
  pinMode(OUT_A, INPUT_PULLUP);
  pinMode(OUT_B, INPUT_PULLUP);    
  attachInterrupt(digitalPinToInterrupt(OUT_A), encoderInt, CHANGE);
  pciSetup(OUT_B);

  //-- Speed sensor setup
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), speedSensorISR, RISING);
//  Timer1.initialize(SAMPLING_TIME);
//  Timer1.attachInterrupt(timerIsr);

  //-- Current sensor setup
  current_sensor.begin();
  pinMode(A0, INPUT);
}

void loop() {  
  static uint64_t prev_sampling(0);
  uint64_t diff_time = micros() - prev_sampling;  
  if(diff_time >= SAMPLING_TIME){
    prev_sampling  = micros();
    noInterrupts();
    vel = (double)ss_counter / (NUM_LATTICE * diff_time * 1e-6);
    ss_counter = 0;
    interrupts();
    
    double analog_read = analogRead(A0);
    ref = map(analog_read, 0, 1024, 0, 140);    
    static double current_pos(.0);
    current_pos = toDeg();      
    
    current = current_sensor.getCurrent_mA();    

    error = ref - vel;  
    in(0) = error;
    out = controller.convolve(in);
    
    in_voltage = out(0);
    if(in_voltage > MAX_VOLTAGE)
      in_voltage = MAX_VOLTAGE;
    else if (in_voltage < -MAX_VOLTAGE)
      in_voltage = -MAX_VOLTAGE;

    writePWM(in_voltage);
    
    StaticJsonDocument<capacity> tdoc;
    tdoc["position"] = String(current_pos);
    tdoc["speed"] = String(vel);  
    tdoc["current"] = String(current);
    tdoc["voltage"] = String(in_voltage);
    tdoc["ref"] = String(ref);
    String serialized_msg;
    serializeJson(tdoc, serialized_msg);  
    sw->sendData(serialized_msg);
//    delay(1);
//    String received_packet = sw->getData();
//    DynamicJsonDocument rdoc(capacity);
//    DeserializationError err = deserializeJson(rdoc, received_packet);
//    if(err){
//  //    Serial.println("Failed to deserialize");
//    }else{
////      String voltage_str = rdoc["voltage"];
////      in_voltage = voltage_str.toDouble();
////      writePWM(in_voltage);
//    }      
    }
}

void serialEvent(){
//  sw->receiveRoutine();  
}

void writePWM(double _voltage){
  //-- Enable the driver
  digitalWrite(EN_R, HIGH);
  digitalWrite(EN_L, HIGH);

  lpwm = .0;
  rpwm = .0;
  if(_voltage < 0){
    lpwm = -_voltage * TO_PWM;
  }else{
    rpwm = _voltage * TO_PWM;
  }
//  const static int MAX_PWM = 10;
//  if(rpwm > MAX_PWM)
//    rpwm = MAX_PWM;
//
//  if(lpwm > MAX_PWM)
//    lpwm = MAX_PWM;

//  Serial.print("LPWM : ");Serial.print(lpwm);Serial.print(" | RPWM : ");Serial.println(rpwm);

  //-- Write PWM
  analogWrite(PWM_R, rpwm);
  analogWrite(PWM_L, lpwm);
}

inline double toDeg(){
  int diff = counter - last_counter;
  static double current_deg(.0);
  current_deg += diff*P2DEG;
  if(current_deg >= 180.)
    current_deg -= 360;
  if(current_deg <= -180.)
    current_deg += 360;

  last_counter = counter;
  return current_deg;
}
