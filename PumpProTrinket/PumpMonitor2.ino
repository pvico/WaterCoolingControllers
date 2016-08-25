//#######################################################
//                                                      #
//             Pro Trinket Pump Controller              #
//                  Philippe Vico 2016                  #
//                                                      #
//#######################################################


#include <TimerOne.h>


// Pin 3 for interrupt, blue wire from EKWB D5 pump
// Pin 2 is not available on the target Trinket Pro board
#define TACH_PIN 3

// Timer 1 PWM needs pin 9 or 10, green wire from EKWB D5 pump
#define PWM_CONTROL_PIN 9

#define BUZZER_PIN 11

// This should normally give a new reading about every 5"
#define NUMBER_OF_PULSES 500

#define PWM_FREQUENCY 25000

#define LOW_RPM 1500
#define THIRTY_SECONDS 30000

// 640 / 1024 = 62%, gives about 3000 RPM
#define PWM_DUTY_CYCLE 640



// This global variable is accessed both by the interrupt service routine
// and the main program.
volatile byte requiredNumberOfPulsesElapsed = 0;

unsigned long lastMicro = 0;
int counter = 0;

// Interrupt service routine
// Within the isr, no other interrupts are allowed
void pumpTachSignalPulse() {
  unsigned long now = micros();

  unsigned long elapsedMicroSecondsSinceLastPulse;

  if(now > lastMicro) { // normal case
    elapsedMicroSecondsSinceLastPulse = now - lastMicro;
  } else { // micros counter has overflowed
    elapsedMicroSecondsSinceLastPulse = (now + ~lastMicro + 1);
  }

  /*
    We need to de-bounce the tach signal. The pump will never exceed 6000RPM
    or 200 Hz signal = 5000us. We reject any pulse that occurs before 1000us after the last one.
    Should the motor be disconnected from the pump top, the RPM will likely far exceed 6000RPM and
    no pulse will be recorded just like for a stopped pump.
  */
  if (elapsedMicroSecondsSinceLastPulse > 1000) {

    if(++counter == NUMBER_OF_PULSES) {
      counter = 0;
      requiredNumberOfPulsesElapsed = 1;
    }

    lastMicro = now;
  }
}


void alertTone(int toneDuration) {
  tone(BUZZER_PIN, 440, toneDuration);
}


void setup() {
  Serial.begin(9600);

  pinMode(BUZZER_PIN, OUTPUT);

  //Play startup notes
  tone(BUZZER_PIN, 440, 100);
  delay(200);
  tone(BUZZER_PIN, 494, 100);
  delay(200);
  tone(BUZZER_PIN, 523, 100);

  Timer1.initialize(1000000 / PWM_FREQUENCY);
  Timer1.pwm(PWM_CONTROL_PIN, PWM_DUTY_CYCLE);

  // We need the pullup as the pump tach signal is open collector
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), pumpTachSignalPulse, FALLING);
}


unsigned long lastTime = 0;

void loop() {
  unsigned long rpm;
  unsigned long now = millis();

  if(requiredNumberOfPulsesElapsed) {
    double durationSinceLastValueAvailable = now - lastTime;
    lastTime = now;
    requiredNumberOfPulsesElapsed=0;

    rpm = (unsigned long)(30. / ((durationSinceLastValueAvailable / NUMBER_OF_PULSES) / 1000));

    Serial.print("Pump RPM: ");
    Serial.println(rpm);

    if(rpm < LOW_RPM) {
      alertTone(1000);
      Serial.println("Pump low RPM!");
    }
  }

  if((now - lastTime) > THIRTY_SECONDS) { // pump stopped
    alertTone(1000);
    delay(1500);
    Serial.println("Pump stopped!");
  }
}
