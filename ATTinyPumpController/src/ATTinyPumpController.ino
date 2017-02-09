#define F_CPU 8000000L

#define RPM_OK_PIN 0    // High level means OK
// #define RPM_CTRL_PIN 2  // Not used yet
#define BUZZER_PIN 1
#define TACH_PIN 4
#define PWM_CONTROL_PIN 3

// This should normally give a new reading about every 5"
#define NUMBER_OF_PULSES 500

#define LOW_RPM 1500
#define THIRTY_SECONDS 30000

#define NOTE_A 27
#define NOTE_B 24
#define NOTE_C 22

#define NORMAL_RPM_OCR1B 65 // 62% duty => 3000 RPM

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Using own main ISO Arduino setup and loop to avoid setup of Timer0 in wiring.c's init function
void mainLoop(); void mainSetup();
int main(void){
  mainSetup();
  while (true) {
    mainLoop();
  }
}

void setPin(uint8_t pin, uint8_t val) {
  if (val == 0) {
    cbi(PORTB, pin);
    // PORTB &= ~(1 << pin);
  } else {
    sbi(PORTB, pin);
    // PORTB |= (1 << pin);
  }
}

int readPin(uint8_t pin) {
  if (PORTB & (1 << pin)) return 1;
	return 0;
}

volatile uint8_t _tonePeriodCounter;
volatile uint8_t _tonePlaying;
volatile uint16_t _toneDurationCounter;
volatile uint16_t _toneDelayCounter;
uint8_t _tonePeriodCount;
uint8_t _alerting;

ISR(TIMER1_COMPB_vect) {
  if(_alerting) {
    if(_toneDurationCounter){
      _toneDelayCounter = 24000;
      if(_tonePeriodCounter == NOTE_A) {
        setPin(BUZZER_PIN, !readPin(BUZZER_PIN));
        _tonePeriodCounter = 0;
      } else {
        _tonePeriodCounter++;
      }
      _toneDurationCounter--;
    } else {
      setPin(BUZZER_PIN, LOW);
      // _tonePeriodCounter = 0;
      if(_toneDelayCounter) {
        _toneDelayCounter--;
      } else {
        _toneDurationCounter = 24000;
      }

    }
  }
  else if(_tonePlaying) {
    if(_toneDurationCounter){
      if(_tonePeriodCounter == _tonePeriodCount) {
        setPin(BUZZER_PIN, !readPin(BUZZER_PIN));
        _tonePeriodCounter = 0;
      } else {
        _tonePeriodCounter++;
      }
      _toneDurationCounter--;
    } else {
      setPin(BUZZER_PIN, LOW);
      _tonePlaying = 0;
    }
  } else {
    setPin(BUZZER_PIN, LOW);
  }
}

// durationMilliseconds must be < 0xFFFF / 24
void playTone(uint8_t periodCount, uint16_t durationMilliseconds) {
  _tonePeriodCount = periodCount;
  uint16_t duration = 24 * durationMilliseconds;
  cli();
  _toneDurationCounter = duration;
  _tonePlaying = 1;
  sei();
}

void waitForNoteFinished() {
    while(true){
      cli();
        if(!_toneDurationCounter){
          sei();
          break;
        }
      sei();
      _delay_ms(1);
    }
}

void setAlerting(uint8_t alert) {
  cli();
  _tonePlaying = 0;
  if(alert){
    if(!_alerting){
      _toneDurationCounter = 24000; // 1"
    }
    _alerting = true;
  } else {
    if(_alerting){
      _alerting = false;
    }
  }
  sei();
}


// This global variable is accessed both by the interrupt service routine
// and the main program.
volatile byte _requiredNumberOfPulsesElapsed = 0;
unsigned long _lastQuarterMilliSecond = 0;
int _counter = 0;
// Interrupt service routine
// Within the isr, no other interrupts are allowed
ISR(PCINT0_vect) {
// void pumpTachSignalPulse() {
  // We react only when the pin is low, ie at the falling edge
  if(!readPin(TACH_PIN)){
    unsigned long now = quarterMilliSecondsSinceStart();
    unsigned long elapsedQuarterMilliSecondSinceLastPulse;
    if(now > _lastQuarterMilliSecond) { // normal case
      elapsedQuarterMilliSecondSinceLastPulse = now - _lastQuarterMilliSecond;
    } else { // QuarterMilliSecond counter has overflowed
      elapsedQuarterMilliSecondSinceLastPulse = (now + ~_lastQuarterMilliSecond + 1);
    }

    /*
    We need to de-bounce the tach signal. The normal speed of the pump is 3000 RPM giving a 50Hz signal.
    The pump will never exceed 6000RPM or 200 Hz signal = 5000us. We reject any pulse that occurs before
    1000us after the last one.
    Should the motor be disconnected from the pump top, the RPM will likely far exceed 6000RPM and
    no pulse will be recorded just like for a stopped pump.
    */
    if (elapsedQuarterMilliSecondSinceLastPulse > 4) {

      if(++_counter == NUMBER_OF_PULSES) {
        _counter = 0;
        _requiredNumberOfPulsesElapsed = 1;
      }

      _lastQuarterMilliSecond = now;
    }
  }
}

// void rpmControlPinChanged(){
//   OCR1B = readPin(RPM_CTRL_PIN) ? NORMAL_RPM_OCR1B : HIGH_RPM_OCR1B;
// }

volatile unsigned long _counter_timer0;
// Called 4000x / sec
ISR(TIMER0_OVF_vect) {
  ++_counter_timer0;  // will overflow after 298h (12 days)
}

unsigned long quarterMilliSecondsSinceStart() {
  cli();
  unsigned long val = _counter_timer0;
  sei();
  return val;
}

unsigned long milliSecondsSinceStart() {
  return quarterMilliSecondsSinceStart() >> 2;
}


void mainSetup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PWM_CONTROL_PIN, OUTPUT);
  pinMode(TACH_PIN, INPUT_PULLUP);
  pinMode(RPM_OK_PIN, OUTPUT);
  // pinMode(RPM_CTRL_PIN, INPUT_PULLUP);
  setPin(RPM_OK_PIN, HIGH);

  _counter_timer0 = 0;
  _tonePeriodCounter = 0;
  _tonePlaying = 0;
  _alerting = 0;
  _toneDurationCounter = 0;
  _tonePeriodCount = 0xFF;
  _toneDelayCounter = 0;


  // Set PWM pump control on PB3
  TCCR1 = _BV(CS11);           // prescaler CK/2
  GTCCR = _BV(COM1B0) | _BV(PWM1B);  // connects OC1B (inverted), pin 3
  OCR1B = NORMAL_RPM_OCR1B;          // the higher the value, the longest the negative pulse
  OCR1C = 170;                  // frequency (24kHz)

  TCCR0B = _BV(CS01);   // clkI/O / 8
  // Enable interrupt for Timer0 on overflow & Timer1 on compare with OCR1B
  cli();
  TIMSK = _BV(OCIE1B) | _BV(TOIE0);
  sei();

  playTone(NOTE_A, 100);
  waitForNoteFinished();
  _delay_ms(100);
  // delay(2000);
  playTone(NOTE_B, 100);
  waitForNoteFinished();
  _delay_ms(100);
  playTone(NOTE_C, 100);
  waitForNoteFinished();
  _delay_ms(2000);
  // We may not use attachInterrupt as it is not defined for PCINT pin change interrupts
  // The interrupt is triggered when the pin toggles
  cli();
  sbi(GIMSK, PCIE);
  sbi(PCMSK, TACH_PIN);
  sei();
}

unsigned long lastTime = 0;
void mainLoop() {
  unsigned long rpm;
  unsigned long now = milliSecondsSinceStart();

  if(_requiredNumberOfPulsesElapsed) {
    double durationSinceLastValueAvailable = now - lastTime;
    lastTime = now;
    _requiredNumberOfPulsesElapsed=0;

    rpm = (unsigned long)(30. / ((durationSinceLastValueAvailable / NUMBER_OF_PULSES) / 1000));

    if(rpm < LOW_RPM) {
      setAlerting(true);
      setPin(RPM_OK_PIN, LOW);
    } else {
      setAlerting(false);
      setPin(RPM_OK_PIN, HIGH);
    }
  }

  if((now - lastTime) > THIRTY_SECONDS) { // pump stopped
    setAlerting(true);
    setPin(RPM_OK_PIN, LOW);
  }
}
