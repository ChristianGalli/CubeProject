// Pin definiti per Arduino Nano per TRE MOTORI
// --- MOTORE 1 ---
const int motor1_pinPWM = 12;     // PWM per Motore 1 (es. ENA su driver L298N)
const int motor1_pinCW = 3;       // Direzione oraria per Motore 1 (es. IN1 su driver L298N)
const int motor1_pinCCW = 2;      // Direzione antioraria per Motore 1 (es. IN2 su driver L298N)

// --- MOTORE 2 ---
const int motor2_pinPWM = 9;      // PWM per Motore 2 (es. ENB su driver L298N) 
const int motor2_pinCW = 5;       // Direzione oraria per Motore 2 (es. IN3 su driver L298N)
const int motor2_pinCCW = 6;      // Direzione antioraria per Motore 2 (es. IN4 su driver L298N)

// --- MOTORE 3 ---
const int motor3_pinPWM = 11;     // PWM per Motore 3 
const int motor3_pinCW = 7;       // Direzione oraria per Motore 3
const int motor3_pinCCW = 8;      // Direzione antioraria per Motore 3

// Analog input pins Velocity
// --- MOTORE 1 ---
const int motor1_analog_vel = A0;

// --- MOTORE 2 ---
const int motor2_analog_vel = A2;

// --- MOTORE 3 ---
const int motor3_analog_vel = A4;

// Analog input pins Averaged Velocity
// --- MOTORE 1 ---
const int motor1_analog_vel_av = A1;

// --- MOTORE 2 ---
const int motor2_analog_vel_av = A3;

// --- MOTORE 3 ---
const int motor3_analog_vel_av = A5;

// Variabili per i comandi dei tre motori
int direction1 = 0;
int speedVal1 = 0;
int direction2 = 0;
int speedVal2 = 0;
int direction3 = 0;
int speedVal3 = 0;

void setup() {
  Serial.begin(115200);

  // Initialize digital output pins
  pinMode(motor1_pinPWM, OUTPUT);
  pinMode(motor1_pinCW, OUTPUT);
  pinMode(motor1_pinCCW, OUTPUT);

  pinMode(motor2_pinPWM, OUTPUT);
  pinMode(motor2_pinCW, OUTPUT);
  pinMode(motor2_pinCCW, OUTPUT);

  pinMode(motor3_pinPWM, OUTPUT);
  pinMode(motor3_pinCW, OUTPUT);
  pinMode(motor3_pinCCW, OUTPUT);

  stopMotors(); // Ensure motor is stopped at start
}

void loop() {
  // Check for serial input commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    parseSerialCommand(input);

    // Applica i comandi ai rispettivi motori
    controlSingleMotor(direction1, speedVal1, motor1_pinCW, motor1_pinCCW, motor1_pinPWM);
    controlSingleMotor(direction2, speedVal2, motor2_pinCW, motor2_pinCCW, motor2_pinPWM);
    controlSingleMotor(direction3, speedVal3, motor3_pinCW, motor3_pinCCW, motor3_pinPWM);
  }

  // Lettura velocità solo quando il motore corrispondente è acceso
  Serial.print("M1_vel: ");
  if (direction1 != 0) { // Motor 1 is active
    float motor1_vel = RPMconv(motor1_analog_vel);
    Serial.print(motor1_vel, 2); // 2 cifre decimali
  } else {
    Serial.print(0,2);
  }
  Serial.print(" RPM | M1_vel_avg: ");
  if (direction1 != 0) { // Motor 1 is active
    float motor1_vel_avg = RPMconv(motor1_analog_vel_av);
    Serial.print(motor1_vel_avg, 2);
  } else {
    Serial.print(0,2);
  }

  Serial.print(" RPM | M2_vel: ");
  if (direction2 != 0) { // Motor 2 is active
    float motor2_vel = RPMconv(motor2_analog_vel);
    Serial.print(motor2_vel, 2);
  } else {
    Serial.print(0,2);
  }
  Serial.print(" RPM | M2_vel_avg: ");
  if (direction2 != 0) { // Motor 2 is active
    float motor2_vel_avg = RPMconv(motor2_analog_vel_av);
    Serial.print(motor2_vel_avg, 2);
  } else {
    Serial.print(0,2);
  }

  Serial.print(" RPM | M3_vel: ");
  if (direction3 != 0) { // Motor 3 is active
    float motor3_vel = RPMconv(motor3_analog_vel);
    Serial.print(motor3_vel, 2);
  } else {
    Serial.print(0,2);
  }
  Serial.print(" RPM | M3_vel_avg: ");
  if (direction3 != 0) { // Motor 3 is active
    float motor3_vel_avg = RPMconv(motor3_analog_vel_av);
    Serial.print(motor3_vel_avg, 2);
  } else {
    Serial.print(0,2);
  }
  Serial.println(" RPM"); // Aggiungi il newline

  delay(50);
}

// === Funzioni ===

// Function to convert analog sensor reading to RPM
float RPMconv(int pin_sens) {
  float valore_bit = analogRead(pin_sens); 
  // forzo la conversione da 0 a 4 perchè altrimenti avrei in automatico una conversione 0 -5 
  float valore = valore_bit * (4.0 / 1023.0);
  // Conversion: 0-4V (Arduino Nano) --> -10000 to 10000 RPM
  // Assuming a sensor setup where 0V maps to -10000 RPM and 4V maps to 10000 RPM.
  float rpm = map(valore*100, 0, 400, -10000, 10000); 

  return rpm;
}

float RPMconv_mot(float vel) {
  int vel_val = vel*100;
  int pwmValue = map(vel_val,0,500,25,230); 
  return pwmValue;
}

void parseSerialCommand(String input) {
  // Utilizziamo strtok per parsa più facilmente i valori separati da virgola
  // La String di Arduino non è compatibile direttamente con strtok, quindi la convertiamo in char array
  char charArray[input.length() + 1];
  input.toCharArray(charArray, sizeof(charArray));

  char *token;

  // Primo token (direzione1)
  token = strtok(charArray, ",");
  if (token != NULL) {
    direction1 = atoi(token);
  } else { Serial.println("Errore parsing dir1"); return; }

  // Secondo token (velocita1)
  token = strtok(NULL, ",");
  if (token != NULL) {
    float speedVal1_cmd = atoi(token);
    speedVal1 = RPMconv_mot(speedVal1_cmd);
  } else { Serial.println("Errore parsing speed1"); return; }

  // Terzo token (direzione2)
  token = strtok(NULL, ",");
  if (token != NULL) {
    direction2 = atoi(token);
  } else { Serial.println("Errore parsing dir2"); return; }

  // Quarto token (velocita2)
  token = strtok(NULL, ",");
  if (token != NULL) {
    float speedVal2_cmd = atoi(token);
    speedVal2 = RPMconv_mot(speedVal2_cmd);
  } else { Serial.println("Errore parsing speed2"); return; }

  // Quinto token (direzione3)
  token = strtok(NULL, ",");
  if (token != NULL) {
    direction3 = atoi(token);
  } else { Serial.println("Errore parsing dir3"); return; }

  // Sesto token (velocita3)
  token = strtok(NULL, ",");
  if (token != NULL) {
    float speedVal3_cmd = atoi(token);
    speedVal3 = RPMconv_mot(speedVal3_cmd);
  } else { Serial.println("Errore parsing speed3"); return; }
}

void controlSingleMotor(int dir, int pwm, int pinCW, int pinCCW, int pinPWM) {
  if (dir == 0) { // Stop
    stopSingleMotor(pinCW, pinCCW, pinPWM);
  } else { // Movimento
    digitalWrite(pinCW, dir > 0 ? HIGH : LOW); // Direzione oraria se dir > 0
    digitalWrite(pinCCW, dir < 0 ? HIGH : LOW); // Direzione antioraria se dir < 0
    analogWrite(pinPWM, pwm); // Applica il PWM
  }
}

void stopSingleMotor(int pinCW, int pinCCW, int pinPWM) {
  digitalWrite(pinCW, LOW);
  digitalWrite(pinCCW, LOW);
  analogWrite(pinPWM, 0); // PWM a 0 per fermare il motore
}

void stopMotors() {
  digitalWrite(motor1_pinCW, LOW);
  digitalWrite(motor1_pinCCW, LOW);
  analogWrite(motor1_pinPWM, 0);

  digitalWrite(motor2_pinCW, LOW);
  digitalWrite(motor2_pinCCW, LOW);
  analogWrite(motor2_pinPWM, 0);

  digitalWrite(motor3_pinCW, LOW);
  digitalWrite(motor3_pinCCW, LOW);
  analogWrite(motor3_pinPWM, 0);
}
