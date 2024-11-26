#include <Wire.h>
#include <Encoder.h>

#define ARDUINO_BOARD

#ifdef ARDUINO_BOARD
#define US_0 2
#define US_1 3
#define US_2 4
#define EN_0_A 5
#define EN_0_B 7
#define EN_1_A 8
#define EN_1_B 12
#define MOT_0_A 6
#define MOT_0_B 9
#define MOT_1_A 10
#define MOT_1_B 11
#define LINE_0 A0
#define LINE_1 A1
#define CURR A2
#define I2C_SDA 
#define I2C_SCL 
#endif


#define MOT_IDLE 127
#define SLAVE_ADDRESS 0x50
#define EEPROM_SIZE 21
#define MOTOR_PULSES_PER_ROT 576

enum US_STATE {
  START = 0,
  WAIT = 1,
  DONE = 2,
};

union EncoderU {
  int32_t value = 0;
  uint8_t bytes[4];
};

union LineU {
  uint16_t value = 0;
  uint8_t bytes[2];
};

union CurrentU {
  uint16_t value = 0;
  uint8_t bytes[2];
};


/*
 * Data Space
 * 
 * 0 - US_0 dist [cm] (8b)
 * 1 - US_1 dist [cm] (8b)
 * 2 - US_2 dist [cm] (8b)
 * 3 - encoder_A [-] (32b)
 * 4   ...
 * 5   ...
 * 6   ...
 * 7 - encoder_B [1] (32b)
 * 8   ...
 * 9   ...
 * 10  ...
 * 11 - current probe [-] (2b)
 * 12  ...
 * 13 - line_sens_0 [-] (2b)
 * 14  ...
 * 15 - line_sens_1 [-] (2b)
 * 16  ...
 * 17 - req_motor_0_speed [-] (1b) (0 - full back, 127 - stop, 255 - full forward) / 127 - 0m/s, 128 - 0.01m/s, 255 - 1.28m/s, 0 - -1.27m/s
 * 18 - req_motor_1_speed [-] (1b) (0 - full back, 127 - stop, 255 - full forward)
 * 19 - reserver
 */
 
uint8_t eeprom[EEPROM_SIZE];
uint8_t address = 0;
uint8_t i2c_write_val = 0;

uint8_t us_pins[3] = {US_0, US_1, US_2};
uint8_t us_state = 0;
unsigned long us_start_time = 0;
uint8_t us_index = 0;
const byte us_eeprom_offset = 0;

 
Encoder enc_0(EN_0_A, EN_0_B);
Encoder enc_1(EN_1_A, EN_1_B);
EncoderU en_0, en_1;

LineU line_0;
LineU line_1;
CurrentU curr;

uint8_t l_motor_pwm = 127;
uint8_t r_motor_pwm = 127;

void setup() {

    Serial.begin (115200);
    while (!Serial) {}

    Wire.begin(SLAVE_ADDRESS);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    for (int i = 0; i < EEPROM_SIZE; i++) {
        eeprom[i] = 0;
    }
    eeprom[17] = 127; // motors stop
    eeprom[18] = 127;
    motor_task();

    Serial.print("Start\n");
}


void loop() {    
    ultrasound_task();
    encoder_task();
    line_task();
    current_task();
    eeprom_task();
    motor_control_task();
    motor_task();
    
    // delay(1);
    delayMicroseconds(100);
}

// Tasks

void ultrasound_task() {

  uint8_t i = us_index;

  switch (us_state) {
  case US_STATE::START: 

    // Serial.println("S");
    pinMode(us_pins[i], OUTPUT);
    digitalWrite(us_pins[i], LOW);
    delayMicroseconds(2);
    
    digitalWrite(us_pins[i], HIGH);
    delayMicroseconds(10);
    
    digitalWrite(us_pins[i], LOW);
    us_start_time = micros();
    pinMode(us_pins[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(us_pins[i]), on_us, CHANGE);
    us_state = US_STATE::WAIT;
    break;

  case US_STATE::WAIT: 
    // Serial.println("W");
    if (micros() - us_start_time > 10000) { // timeout (about 1m max dist)
      detachInterrupt(digitalPinToInterrupt(us_pins[i]));
      eeprom[us_eeprom_offset + i] = 255;
      us_state = US_STATE::DONE;  
    }
    break;

  case US_STATE::DONE:
    // Serial.println("D");
    us_state = US_STATE::START;
    us_index = (us_index + 1) % 3;
    break;
  }
}

void encoder_task() {
  en_0.value = enc_0.read();
  en_1.value = enc_1.read();
}

void line_task() {
  line_0.value = analogRead(LINE_0);
  line_1.value = analogRead(LINE_1);
}

void current_task() {
  curr.value = analogRead(CURR);
}


void eeprom_task() {
  noInterrupts();

  eeprom[3] = en_0.bytes[3];
  eeprom[4] = en_0.bytes[2];
  eeprom[5] = en_0.bytes[1];
  eeprom[6] = en_0.bytes[0];

  eeprom[7] = en_1.bytes[3];
  eeprom[8] = en_1.bytes[2];
  eeprom[9] = en_1.bytes[1];
  eeprom[10] = en_1.bytes[0];
  
  eeprom[11] = curr.bytes[1];
  eeprom[12] = curr.bytes[0];

  eeprom[13] = line_0.bytes[1];
  eeprom[14] = line_0.bytes[0];

  eeprom[15] = line_1.bytes[1];
  eeprom[16] = line_1.bytes[0];

  interrupts();
}

void motor_task() {
  if (l_motor_pwm < MOT_IDLE) {
        analogWrite(MOT_0_A, 0);
        analogWrite(MOT_0_B, (MOT_IDLE - l_motor_pwm) * 2);
  } else if (l_motor_pwm == MOT_IDLE) {
      analogWrite(MOT_0_A, 0);
      analogWrite(MOT_0_B, 0);
  } else {
      analogWrite(MOT_0_A, (l_motor_pwm - 127) * 2);
      analogWrite(MOT_0_B, 0);
  }

  if (r_motor_pwm < MOT_IDLE) {
      analogWrite(MOT_1_A, 0);
      analogWrite(MOT_1_B, (MOT_IDLE - r_motor_pwm) * 2);
  } else if (r_motor_pwm == MOT_IDLE) {
      analogWrite(MOT_1_A, 0);
      analogWrite(MOT_1_B, 0);
  } else {
      analogWrite(MOT_1_A, (r_motor_pwm - MOT_IDLE) * 2);
      analogWrite(MOT_1_B, 0);
  }
}

class LowPass {
  public:
    LowPass(float coef) : coef_{coef} {}

    float step (float x) {
      val_ = coef_ * val_ + (1.0 - coef_) * x;
      return val_;
    }

  private:
    float val_ = 0;
    float coef_ = 0.0;
};

class Speeder {
  public:
  float step(int32_t encoder, float dt) {
    if (first_) {
      first_ = false;
      last_encoder_ = encoder;
      return 0.0;
    }
    int32_t enc_diff = ((encoder - last_encoder_));
    last_encoder_ = encoder;
    return -((float)(enc_diff)) / dt / MOTOR_PULSES_PER_ROT * 6.28 * 0.033;
  }
  private:
  bool first_ = true;
  float last_encoder_ = 0;
};

class PID {
  public: 
  PID(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }

  int8_t step (float speed, float setpoint, float dt) {

    float err = setpoint - speed;

    last_err = err;

    integral += err * dt;
    if (integral * ki > max_output) {
      integral = max_output / ki;
    }
    else if (integral * ki < -max_output) {
      integral = -max_output / ki;
    }

    float deriv = (err - last_err) / dt;
    float output = kp * err + ki * integral + kd * deriv;
    integral *= 0.99;
    return constrain((int)output, -max_output, max_output);
  }
  private:

  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;
  float integral = 0.0;
  float last_err = 0.0;
  float max_output = 127;
};

class MotorPipeline {
  
  static constexpr float KP = 4000.0;
  static constexpr float KI = KP * 0.5f; //1000.0;
  static constexpr float KD = KP * 0.25f; //500.0;

  public:
  MotorPipeline() : pid_{KP, KI, KD}, low_pass_(0.9) {}

  float step(int32_t enc, float setpoint, float dt) {
    float spd = speeder_.step(enc, dt);
    float spd_lp = low_pass_.step(spd);
    // Serial.print(setpoint);
    // Serial.print(", ");
    // Serial.print(spd);
    // Serial.print(", ");
    // Serial.print(spd_lp);
    // Serial.print("\n");
    // printf("%f, %f, %f\n", setpoint, spd, spd_lp);
    float output = pid_.step(spd_lp, setpoint, dt);
    return output;
  }

  private:
  Speeder speeder_;
  LowPass low_pass_;
  PID pid_;
};


MotorPipeline mot_pipeline_l;
MotorPipeline mot_pipeline_r;

void motor_control_task() {

  static unsigned long last_time = millis();
  float setpoint_l = float(eeprom[17] - 127) / 100.0; //0.5 * sin(millis()/1000.0); 
  float setpoint_r = float(eeprom[18] - 127) / 100.0; 
  
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - last_time;

  if (elapsed_time >= 5) {
    float dt = (float) elapsed_time / 1000.0;

    l_motor_pwm = 127 + mot_pipeline_l.step(en_0.value, setpoint_l, dt);
    r_motor_pwm = 127 - mot_pipeline_r.step(en_1.value, -setpoint_r, dt);
    last_time = current_time;
  }
}


// Interrupts

void on_us() {
  uint8_t i = us_index;
  if(digitalRead(us_pins[i]) == HIGH) {
    us_start_time = micros();
  } else if (digitalRead(us_pins[i]) == LOW) {
    uint8_t distance = (micros() - us_start_time) / 58;
    eeprom[us_eeprom_offset + i] = distance;
    detachInterrupt(digitalPinToInterrupt(us_pins[i]));
      us_state = US_STATE::DONE;  
  }
}


uint8_t i2c_val = 0;

void requestEvent() {
    Wire.write(eeprom[address]);
}


void receiveEvent(int bytes) {
    if (bytes == 1) {
        address = Wire.read();
    } else if (bytes == 2) {
        address = Wire.read();
        eeprom[address] = Wire.read();
    }
}
