#include <LiquidCrystal.h>
#include <Servo.h>
#include <HX711.h>

#define INTERRUPT_PIN 0 // this is pin2 on Nano
#define ESC_PIN 3

#define SERIALUPDATE_INTERVAL 1000   // millis between serial updates.
#define PROGNAME "Prop Tester, v1.04"
#define LOADCELL_CALIBRATION 322.425f

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(4, 5, 6, 7, 8, 9);
Servo esc;
HX711 scale(10,11);


double watthours = 0, watts = 0, volts = 0, amps = 0, thrust = 0;
int servo = 950; 
unsigned int magnetic_poles = 12; // Set this to the # of poles in your motor.
unsigned int rpm = 0;             // See calculation below.
unsigned long commutation_period = 0;  // the RPM sensor period in microseconds.
unsigned long last_interrupt = 0;      // millis() of the last interrupt that fired.

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void(* resetFunc) (void) = 0;

void setup() {
  lcd.begin(20, 4);
  lcdWriteStringAtPos(0, 0, PROGNAME);
  lcdWriteStringAtPos(9, 3, "Booting");
  for (int i=16; i<20; i++) {
    lcdWriteStringAtPos(i, 3 ,".");
    delay(500);
  }
  lcd.clear();

  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(INTERRUPT_PIN, commutation_interrupt, RISING);

  esc.attach(ESC_PIN);
  esc.writeMicroseconds(servo);

  inputString.reserve(80);

  scale.set_scale(LOADCELL_CALIBRATION);
  scale.tare(50);

  Serial.begin(9600);
  Serial.println("# "PROGNAME);
  doCommand_help();
}

/* Note: micros() has a 4uS resoluation on Arduino, may want
   to take multiple measurements and return median.
   Note: we need to save timestamp of the last interrupt, in
   order to clear the RPM/commutation_period  when the motor 
   stops and we no longer receive interrupts,
 */
void commutation_interrupt()
{
  static unsigned long last_micros = 0;
  unsigned long now = micros();

  last_interrupt = millis();
  if (last_micros == 0 || now < last_micros) {
    last_micros = now;
    return;
  }
  commutation_period = now - last_micros;
  last_micros = now;
  return;
}

void lcdUpdate()
{
  char *p;

  lcdWriteDoubleAtPos(0, 0, volts);
  lcdWriteStringAtPos(5, 0, "V");
  lcdWriteDoubleAtPos(6, 0, amps);
  lcdWriteStringAtPos(11, 0, "A");
  lcdWriteDoubleAtPos(13, 0, watts);
  lcdWriteStringAtPos(18, 0, "W");

  lcdWriteIntAtPos(0, 1, servo);
  lcdWriteStringAtPos(5, 1, "uS");
  lcdWriteDoubleAtPos(13, 1, watthours);
  lcdWriteStringAtPos(18, 1, "Wh");

  lcdWriteIntAtPos(0, 3, rpm);
  lcdWriteStringAtPos(6, 3, "RPM");
  lcdWriteIntAtPos(14, 3, thrust);
  lcdWriteStringAtPos(19, 3, "g"); 
}

void serialUpdate()
{
  static unsigned long next_update = millis() + SERIALUPDATE_INTERVAL;
  static int updates_sent = 0;

  if (millis() < next_update)
    return;

  if (updates_sent % 10 == 0)
    Serial.println("# millis,volts,amps,watts,watthours,servo,commutation_period,rpm,magnetic_poles,thrust,A0,A1,A2,A3");
  Serial.print(millis()); 
  Serial.print(",");
  Serial.print(volts); 
  Serial.print(",");  
  Serial.print(amps); 
  Serial.print(",");
  Serial.print(watts); 
  Serial.print(",");
  Serial.print(watthours); 
  Serial.print(",");
  Serial.print(servo); 
  Serial.print(",");
  Serial.print(commutation_period); 
  Serial.print(",");
  Serial.print(rpm); 
  Serial.print(",");
  Serial.print(magnetic_poles); 
  Serial.print(",");
  Serial.print(thrust); 
  Serial.print(",");
  Serial.print(analogRead(A0)); 
  Serial.print(",");
  Serial.print(analogRead(A1)); 
  Serial.print(",");
  Serial.print(analogRead(A2)); 
  Serial.print(",");
  Serial.print(analogRead(A3)); 
  Serial.println();
  updates_sent++;
  next_update = millis() + SERIALUPDATE_INTERVAL;
}

void measureUpdate() {
  static unsigned long last_measurement = 0;

  /* A1, amps, reads 0 for -30A, 512 for 0A, 1023 for +30A, ACS712 */
  long amps_tmp = 512 - analogRead(A1);
  /* In our tacho application, current can only be positive */
  if (amps_tmp < 0) amps_tmp = 0;
  amps = ((double) amps_tmp * 30.0) / 512;

  /* A2, voltage, reads 477 at 15.52V, 1K:1K+4.7K voltage divider */
  volts = ((double) analogRead(A2) / 477) * 15.52;

  /* Measure the thrust on the loadcell via hx711 */
  thrust = scale.get_units(5);
  if (thrust < 0)
    thrust = 0;

  if (last_measurement == 0) {
    last_measurement = millis();
    return;
  }

  /* Integrate watthours in the last measurement period */
  watts = amps * volts;  
  watthours = watthours + watts * (double)((millis() - last_measurement)) / 1000 / 3600;

  /* The commutation_period is calculated within the commutations_interrupt routine, but
   if no interrupts occur (motor is off), then we'll have to reset the period manually */
  if (millis() > last_interrupt + 20) {
    commutation_period = 0;
    rpm = 0;
  } 
  else {
    /* Note on math: there are 'magnetic_poles' commutations per full revolution
       so if one commutation takes 1000uS, we will be doing 1000 commutations/sec
       which is 60*1000 commutations/min, which is 60/magnetic_poles * 1000 RPM
     */
    rpm = 60/magnetic_poles*1000000/commutation_period;
  }

  last_measurement = millis();
}

void doCommand_reset()
{    
  Serial.println("# Resetting");
  lcd.clear();
  Serial.flush();
  resetFunc();
}

void doCommand_help()
{
  Serial.println("# Commands");
  Serial.println("# help     : This help screen");
  Serial.println("# reset    : Reset the arduino");
  Serial.println("# tare     : Reset the loadcell to 0"); 
  Serial.println("# esc N    : Set the PWM output to N [950..2100])");
  Serial.println("# poles N  : Set number of magnetic poles to N [2..20]");
}

void doCommand_esc(String command)
{
  int spaceIndex = command.indexOf(' ');
  int value;
  if (spaceIndex < 0) {
    Serial.println("# Error, no argument found");
    return;
  }
  value = command.substring(spaceIndex+1).toInt();
  if (value < 950 || value > 2100) {
    Serial.print("# Error, value must be between 950 and 2100 inclusive, you wrote '");
    Serial.print(value);
    Serial.println("'");
    return;
  }
  Serial.print("# Setting ESC output to ");
  Serial.println(value);
  servo = value;
  esc.writeMicroseconds(servo);
}

void doCommand_poles(String command)
{
  int spaceIndex = command.indexOf(' ');
  int value;
  if (spaceIndex < 0) {
    Serial.println("# Error, no argument found");
    return;
  }
  value = command.substring(spaceIndex+1).toInt();
  if (value < 2 || value > 20) {
    Serial.print("# Error, value must be between 2 and 20 inclusive, you wrote '");
    Serial.print(value);
    Serial.println("'");
    return;
  }
  Serial.print("# Setting magnetic_poles to ");
  Serial.println(value);
  magnetic_poles = value;
}

void doCommand_tare()
{
  Serial.println("# Setting thrust to zero");
  scale.tare();
}

void doCommand(String command)
{
  if (command.startsWith("help")) {
    doCommand_help();
    return;
  }
  if (command.startsWith("reset")) {
    doCommand_reset();
    return;
  }
  if (command.startsWith("esc")) {
    doCommand_esc(command);
    return;
  }
  if (command.startsWith("tare")) {
    doCommand_tare();
    return;
  }
  if (command.startsWith("poles")) {
    doCommand_poles(command);
    return;
  }
  if (command.length() == 0) {
    Serial.println ("# Hello World");
    return;
  }
  Serial.print("# Unknown input: \"");
  Serial.print(command); 
  Serial.print("\", ");
  Serial.print(command.length()); 
  Serial.println(" chars; type 'help' for help");
}


void loop() {
  measureUpdate();
  lcdUpdate();
  serialUpdate();
  if (stringComplete) {
    doCommand(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  delay(50);
}


/*
 SerialEvent occurs whenever new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } 
    else {
      // add it to the inputString:
      inputString += inChar;
    }
  }
}

/* Some LCD helper functions */
void lcdWriteStringAtPos(const int x, const int y, const char* s) {
  int offset = x * y;
  lcd.setCursor(x,y);
  lcd.print(s);
  return;
}

void lcdWriteIntAtPos(const int x, const int y, const long i) {
  char charBuf[10];
  size_t len;
  char *p;
  snprintf(charBuf, 6, "%u", i);
  /* Now pad the buffer with space */
  len = strlen(charBuf);
  p = &charBuf[len];
  while (len < 5) {
    *p++ = ' ';
    len++;
  }
  *p = '\0';
  lcdWriteStringAtPos(x, y, charBuf);
  return;
}

void lcdWriteDoubleAtPos(const int x, const int y, const double f) {
  char charBuf[6];
  if (f<=-100) {
    dtostrf(f, 5, 0, charBuf);
    charBuf[5] = 0;
  } 
  else if (f<=-10) 
    dtostrf(f, 5, 1, charBuf);
  else if (f<10)
    dtostrf(f, 5, 2, charBuf);
  else if (f<100)
    dtostrf(f, 5, 1, charBuf);
  else  
    dtostrf(f, 5, 0, charBuf);

  lcdWriteStringAtPos(x,y,charBuf);
  return;
}







