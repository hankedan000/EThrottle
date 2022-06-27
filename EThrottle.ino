#include <PID_v1.h>
#include <string.h>
#include <stdlib.h>

#define TPS_N_PIN 4 // non-inverted TPS pin
#define TPS_I_PIN 5 // inverted TPS pin

// the highest setpoint the throttle PID controller can target
// this is the highest TPS sensor reading we get if you manually
// open the throttle to "wide open".
#define MAX_SETPOINT 856

#define MOTOR_P 3
#define MOTOR_N 4

uint16_t tpsN = 0;
uint16_t tpsI = 0;

double pidSetpoint = 0;
double pidIn = 0;
double pidOut = 0;

//Specify the links and initial tuning parameters
double Kp=6.0, Ki=20.0, Kd=0.0;
PID pid(&pidIn,&pidOut,&pidSetpoint,Kp,Ki,Kd, DIRECT);

void setup() {
  // help reduce buzzing sounds from throttle body
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz

  pinMode(TPS_N_PIN, INPUT);
  pinMode(TPS_I_PIN, INPUT);

  pinMode(MOTOR_P, OUTPUT);
  pinMode(MOTOR_N, OUTPUT);

  digitalWrite(MOTOR_P, LOW);
  digitalWrite(MOTOR_N, LOW);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 255);
  pid.SetSampleTime(10);
  pidSetpoint = 400;
  
  Serial.begin(115200);
}

size_t serBuffIdx = 0;
char serBuff[20] = "400";
char serCmd = '\0';
bool logPID_In = false;

void logPID_Params() {
  Serial.print("Kp = " );
  Serial.print(pid.GetKp());
  Serial.print(", Ki = " );
  Serial.print(pid.GetKi());
  Serial.print(", Kd = " );
  Serial.println(pid.GetKd());
}

// s### -> set 'pidSetpoint' to ###
// p### -> set 'Kp' to ###
// i### -> set 'Ki' to ###
// d### -> set 'Kd' to ###
// l    -> toggle logging of 'pidIn'
// P    -> print current PID parameters
void doSerial() {
  while (Serial.available())
  {
    if (serCmd == '\0') {
      serCmd = Serial.read();
      serBuffIdx = 0;
    } else {
      serBuff[serBuffIdx] = Serial.read();
      if (serBuff[serBuffIdx] == '\n') {
        serBuff[serBuffIdx] = '\0';// null terminate for atoi() to work
        double val = 0;
        if (serBuffIdx > 0) {
          val = atof(serBuff);
        }
        switch (serCmd) {
          case 's':
            pidSetpoint = val;
            Serial.print("pidSetpoint = ");
            Serial.println(val);
            break;
          case 'p':
            Kp = val;
            break;
          case 'i':
            Ki = val;
            break;
          case 'd':
            Kd = val;
            break;
          case 'l':
            logPID_In = ! logPID_In;
            break;
          case 'P':
            logPID_Params();
            break;
        }

        if (serCmd == 'p' || serCmd == 'i' || serCmd == 'd') {
          pid.SetTunings(Kp,Ki,Kd);
          logPID_Params();
        }

        serCmd = '\0';// restart command parsing
      }
      serBuffIdx++;
    }
  }
}

void loop() {
  doSerial();
//  tpsN = analogRead(TPS_N_PIN);
  tpsI = analogRead(TPS_I_PIN);

  // sanitize PID inputs
  if (pidSetpoint > MAX_SETPOINT) {
    pidSetpoint = MAX_SETPOINT;
  }

  pidIn = tpsI;
  pid.Compute();

  if (logPID_In) {
    Serial.print("pidIn:");
    Serial.println(pidIn);
  }

  analogWrite(MOTOR_P, pidOut);
}
