#include <EEPROM.h>

const int MbSenseOut = 11;     // yellow. Either Timer2 on pin 11, or to copy FanA RPM, use A_ISR, CHANGE
const int MbControlIn = 8;    // blue
const int FanASenseIn = 2;    // only 2 an 3 on Nano, yellow
const int FanAControlOut =  9; // blue
const int FanBSenseIn = 3;
const int FanBControlOut =  10;
const int pwmCPUsmoothing = 15; // with 1 sec loop delay = 15 sec

volatile unsigned long timeA, timeB, timeISR; // ISR variables will change (volatile):
volatile int fanRpms[2] = {};

int curveIn[] = { 0, 10, 20, 30, 40,  50,  60,  70,  80,  90, 100};  // len 11
int curveRpmA[] = { 0,  0,  10,  15,  20,  25,  35,  45,  60,  80, 100};
int curveRpmB[] = { 0,  0,  10,  15,  20,  25,  35,  45,  60,  80, 100};
int fanPins[2] = { FanAControlOut, FanBControlOut };
int pwmCPUarray[ pwmCPUsmoothing ] = {};
int fanRpmNormalized[2][11] = { {}, {} };

byte MbControlIn_mask;
volatile byte *MbControlIn_port;

void A_ISR() {
    timeISR = micros();
    fanRpms[0] = 30000000 / ( timeISR - timeA ); // f=1/T  rpm=f*60/2  rpm=30e6/T
    timeA = timeISR;
    // copy RPM signal to to MbSenseOut. Change ISR to CHANGE
    // digitalWrite(MbSenseOut, digitalRead(FanASenseIn) );
}

void B_ISR() {
    timeISR = micros();
    fanRpms[1] = 30000000 / ( timeISR - timeB );
    timeB = timeISR;
}

// https://arduino.stackexchange.com/questions/25609/set-pwm-frequency-to-25-khz/25623#25623
void analogWrite25k(int pin, int value)
{
    switch (pin) {
        case 9:
            OCR1A = value;
            break;
        case 10:
            OCR1B = value;
            break;
        default:
            break; // no other pin will work
    }
}

// https://playground.arduino.cc/Main/MultiMap
int multiMap(int val, int* _in, int* _out, uint8_t size)
{
    if (val <= _in[0]) return _out[0];
    if (val >= _in[size-1]) return _out[size-1];
    uint8_t pos = 1;
    while(val > _in[pos]) pos++;
    if (val == _in[pos]) return _out[pos];
    return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

void calibrateFans () {
    int eepromAddr = 0;
    bool isCalibrated [ 2 ] = {};
    bool isRunning [ 2 ] = {};
    bool calibrating [ 2 ] = {};
    int fanPwmRpm[2][11] = { {}, {} };
    long maxRpm [2] = { 0, 0 };
    long minRpm [2] = { 10000, 10000 };
    float norm;

    analogWrite25k(fanPins[0], 0);  // slow down before calibration
    analogWrite25k(fanPins[1], 0);
    delay(2000);  // wait for RPM data

    for (int fan = 0; fan < 2; fan++) {
        // EEPROM.write(200 + fan, 0);    //  debug
        isCalibrated[fan] = ( EEPROM.read(200 + fan) == 1 );
        isRunning[fan] = ( fanRpms[fan] > 0 );
        if ( isCalibrated[fan] != isRunning[fan] ) {
            if ( isRunning[fan] ) {   // .. but not calibrated
                calibrating[fan] = true;
                EEPROM.write(200 + fan, 1);    // Set marker
                Serial.print("Found new fan. Calibration started fan: ");
                Serial.println(fan);
            }
            else {
                EEPROM.write(200 + fan, 0);    // Erase marker
                Serial.print("Fan not detected, Removing fan: ");
                Serial.println(fan);
            }
        }
    }

    for (int pwm = 0; pwm < 11; pwm++) {
        for (int fan = 0; fan < 2; fan++) { // separated to do both fans at same time
            if ( calibrating[fan] )  {
                analogWrite25k(fanPins[fan], 3200 * pwm / 100);
                delay(1000);     // 3000, XXX debug // spin up fan(s) to next pwm value
            }
        }
        for (int fan = 0; fan < 2; fan++) {
            if ( !calibrating[fan] ) continue;
            fanPwmRpm[fan][pwm] = fanRpms[fan];
            if ( fanPwmRpm[fan][pwm] > maxRpm[fan] ) { maxRpm[fan] = fanPwmRpm[fan][pwm]; }
            if ( fanPwmRpm[fan][pwm] < minRpm[fan] ) { minRpm[fan] = fanPwmRpm[fan][pwm]; }
        }
    }

    for (int fan = 0; fan < 2; fan++) {   // write top-normalised, min to 100, profiles to eeprom. Separated cuz need min max
        Serial.print("Fan:");
        Serial.println(fan);
        for (int pwm = 0; pwm < 11; pwm++) {
            Serial.print("  PWM:");
            Serial.print(pwm * 10);
            if ( calibrating[fan] ) {
                norm = 100 * (float) fanPwmRpm[fan][pwm] / (float) maxRpm[fan];
                fanRpmNormalized[fan][pwm] = norm;
                EEPROM.write(fan * 100 + pwm, fanRpmNormalized[fan][pwm]);
                Serial.print("  RPM:");
                Serial.print(fanPwmRpm[fan][pwm]);
            }
            else if ( isRunning[fan] ) {
                fanRpmNormalized[fan][pwm] = EEPROM.read(fan * 100 + pwm);
            }
            Serial.print("  %max:");
            Serial.print(fanRpmNormalized[fan][pwm]);
            Serial.println("");
        }
    }

    Serial.println("Calibration complete");
}

void setup()
{
    Serial.begin(19200);
    while (!Serial) { ; }
    Serial.println("Serial Connection OK.");
    // Configure Timer 1 for PWM @ 25 kHz.
    TCCR1A = 0;           // undo the configuration done by...
    TCCR1B = 0;           // ...the Arduino core library
    TCNT1  = 0;           // reset timer
    TCCR1A = _BV(COM1A1)  // non-inverted PWM on ch. A
           | _BV(COM1B1)  // same on ch; B
           | _BV(WGM11);  // mode 10: ph. correct PWM, TOP = ICR1
    TCCR1B = _BV(WGM13)   // ditto
           | _BV(CS10);   // prescaler = 1
    ICR1   = 320;         // TOP = 320

    pinMode(MbSenseOut, OUTPUT);
    pinMode(MbControlIn, INPUT);
    pinMode(FanASenseIn, INPUT_PULLUP);
    pinMode(FanBSenseIn, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FanASenseIn), A_ISR, FALLING); // Attach an interrupt to the ISR vector
    attachInterrupt(digitalPinToInterrupt(FanBSenseIn), B_ISR, FALLING); // Attach an interrupt to the ISR vector
    pinMode(FanAControlOut, OUTPUT); // Set the PWM pins as output.
    pinMode(FanBControlOut, OUTPUT);

    MbControlIn_mask = digitalPinToBitMask(MbControlIn);
    MbControlIn_port = portInputRegister(digitalPinToPort(MbControlIn));

    // TIMER 2 for frequency 1500 Hz:
    TCCR2A = 0; // set entire TCCR2A register to 0
    TCCR2B = 0; // same for TCCR2B
    // turn on CTC, top = OCR2A mode, turn off B output on pin3, Toggle-on-Compare for A on pin 11
    TCCR2A |= (1 << WGM21) | (1 << COM2A0);  
    TCCR2B |= (1 << CS22) | (1 << CS21) | (0 << CS20); // Set bits for 256 prescaler
    TCNT2  = 0; // initialize counter value to 0
    // set compare match register for 1500 Hz increments
    OCR2A = 41; // = 16000000 / (256 * 1500) - 1 (must be <256)

    calibrateFans();
}

void loop()
{
    unsigned long countUp, countDown, countErr;
    int pwmCPU, pwmA, pwmB, DesiredRpmA, DesiredRpmB;
    int pwmCPUavg = 0;
    countUp = 0;
    countDown = 0;
    countErr = 0;

    for (int i = 0; i < 3000; i++) {
        if ( (*MbControlIn_port & MbControlIn_mask) != 0) { countUp++; }
        else if ( (*MbControlIn_port & MbControlIn_mask) == 0) { countDown++; }
        // else { countErr++; }
        delayMicroseconds(100); // ~0.3 sec 
    }

    pwmCPU = (countUp * 100) / (countUp + countDown); 

    // Buffer and shift 15 previous values

    for (int i = pwmCPUsmoothing-1; i > 0; i--) {
        pwmCPUavg += pwmCPUarray[i];
        pwmCPUarray[i] = pwmCPUarray[i-1];
    }
    pwmCPUarray[0] = pwmCPU;
    pwmCPU = pwmCPUavg / pwmCPUsmoothing;

    if ( micros() - timeA > 300000 ) { fanRpms[0] = 0; } // no tacho signal or below 100rpm
    if ( micros() - timeB > 300000 ) { fanRpms[1] = 0; }

    DesiredRpmA = multiMap(pwmCPU, curveIn, curveRpmA, 11);
    DesiredRpmB = multiMap(pwmCPU, curveIn, curveRpmB, 11);

    pwmA = multiMap(DesiredRpmA, fanRpmNormalized[0], curveIn, 11); // that was hard :)
    pwmB = multiMap(DesiredRpmB, fanRpmNormalized[1], curveIn, 11);
    
    Serial.print("pwmCPU:");
    Serial.print(pwmCPU);
    Serial.print("  desiredA:");
    Serial.print(DesiredRpmA);
    Serial.print("  pwmA:");
    Serial.print(pwmA);
    Serial.print("  rpmA:");
    Serial.print(fanRpms[0]);
    Serial.print("  pwmB:");
    Serial.print(pwmB);
    Serial.print("  rpmB:");
    Serial.println(fanRpms[1]);

    analogWrite25k(FanAControlOut, 320 * pwmA / 100);
    analogWrite25k(FanBControlOut, 320 * pwmB / 100);
    delay(1000);
}
