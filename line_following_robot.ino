//defining motor pins
int enable_1= 5;
int enable_2= 11;
int out_1= 12;
int out_2= 10;
int out_3= 9;
int out_4= 8;

//values of ir to store after calibration
long minir_1=0;
long minir_2=0;
long minir_3=0;
long maxir_1=0;
long maxir_2=0;
long maxir_3=0;

//defining ir pins
#define ir_1 A0
#define ir_2 A1
#define ir_3 A2

//defining slider pin 
int slider_1=6;
int slider_2=7;

//defining led pin
int led=13;

//definign PID variables
float Kp=1.2;
float Ki=0;
float Kd=0.1;
float integral=0;
//defining other variables
float lastError=0;
//variable to store the delay in time
int calibration_wait=300;
int finish_calibration=100;

// more motor variables
int baseSpeed=200;
float calibration(int ir)
{
  float sum=0;
  for(int i=0;i<500;i++){
    sum+=analogRead(ir);
    delayMicroseconds(100);
  }
  sum/=500;
  return sum;
}
void stop_motor(){
  analogWrite(enable_1,0);
  analogWrite(enable_2,0);
}
void blink(int time){
  for (int i=0;i<17;i++){
  digitalWrite(led,LOW);
  delay(time);
  digitalWrite(led,HIGH);
  delay(time);}
}
void setup() {
  // 1. Configure Pins
  pinMode(enable_1, OUTPUT);
  pinMode(enable_2, OUTPUT);
  pinMode(out_1, OUTPUT);
  pinMode(out_2, OUTPUT);
  pinMode(out_3, OUTPUT);
  pinMode(out_4, OUTPUT);

  pinMode(ir_1, INPUT);
  pinMode(ir_2, INPUT);
  pinMode(ir_3, INPUT);

  pinMode(slider_1, INPUT_PULLUP); // Pin 6 (Calibrate Side)
  pinMode(slider_2, INPUT_PULLUP); // Pin 7 (Run Side)
  pinMode(led, OUTPUT);

  stop_motor();
  Serial.begin(9600);

  // 2. Initialize Variables 
  // We start Min at 1023 and Max at 0 so they update immediately upon reading
  minir_1 = 1023; maxir_1 = 0;
  minir_2 = 1023; maxir_2 = 0;
  minir_3 = 1023; maxir_3 = 0;

  // 3. CHECK FOR CALIBRATION MODE
  // If the switch is on the "Calibrate" side (Pin 6 connected to GND/LOW)
  if (digitalRead(slider_1) == LOW) {
    
    Serial.println("--- CALIBRATION MODE ---");
    Serial.println("Slide the robot back and forth over the line!");
    
    // LOOP: Keep calibrating AS LONG AS the switch is on Pin 6
    while (digitalRead(slider_1) == LOW) {
      
      // A. Blink LED fast to show "I am Recording!"
      digitalWrite(led, !digitalRead(led)); // Toggles LED on/off
      
      // B. Update Sensors (Find Extremes)
      calibrateSensors(); 
      
      delay(50); // Fast blink speed
    }
    
    // Switch was flipped to Run! Calibration is done.
    digitalWrite(led, LOW); 
    Serial.println("Calibration Saved.");
    
    // Optional: Print values to prove it worked
    Serial.print("L: "); Serial.print(minir_1); Serial.print("-"); Serial.println(maxir_1);
    Serial.print("M: "); Serial.print(minir_2); Serial.print("-"); Serial.println(maxir_2);
    Serial.print("R: "); Serial.print(minir_3); Serial.print("-"); Serial.println(maxir_3);
  }

  // 4. WAIT FOR RUN
  // If the switch is floating (neither side), wait until it hits Run (Pin 7 LOW)
  while (digitalRead(slider_2) == HIGH) {
     stop_motor();
  }
  
  // Now entering loop()...
}

// --- NEW CALIBRATION FUNCTION ---
void calibrateSensors() {
  // Read current values
  int val1 = analogRead(ir_1);
  int val2 = analogRead(ir_2);
  int val3 = analogRead(ir_3);

  // Update Sensor 1 (Left)
  if (val1 < minir_1) minir_1 = val1; // Found a new lowest White
  if (val1 > maxir_1) maxir_1 = val1; // Found a new highest Black

  // Update Sensor 2 (Mid)
  if (val2 < minir_2) minir_2 = val2;
  if (val2 > maxir_2) maxir_2 = val2;

  // Update Sensor 3 (Right)
  if (val3 < minir_3) minir_3 = val3;
  if (val3 > maxir_3) maxir_3 = val3;
}
float irRead(){
  // read values of each ir sensor
  float read_1=analogRead(ir_1);
  float read_2=analogRead(ir_2);
  float read_3=analogRead(ir_3);
  read_1=constrain(map(read_1,minir_1,maxir_1,199,255),199,255);
  read_2=constrain(map(read_2,minir_2,maxir_2,199,255),199,255);
  read_3=constrain(map(read_3,minir_3,maxir_3,199,255),199,255);

  //map given values to a max and min and constrain to clear any noise issues.
  
  float mapir_1=constrain(map(read_1,minir_1,maxir_1,0,1000),0,1000);
  float mapir_2=constrain(map(read_2,minir_2,maxir_2,0,1000),0,1000);
  float mapir_3=constrain(map(read_3,minir_3,maxir_3,0,1000),0,1000);
  
  //doing weighted mean calcualtions
  float denominator = mapir_1+mapir_2+mapir_3;
  float currentError=0;
  //check if denominator > 0, this occues when robot fell off the line and needs to recorrect.
  if (denominator > 100) { 
    // Only do the heavy math if we actually see a line
    currentError = (mapir_1 * -1000 + mapir_2 * 0 + mapir_3 * 1000) / denominator;
} else {
    // We are off the line! Use the memory (lastError)
    currentError = lastError; 
}
  return currentError;
}

float pid(){
  float error=irRead();
  float derivative = error-lastError;
  integral+=error;
  integral=constrain(integral,-2000,2000);
  float turn = (Kp*error)+(Ki*integral)+(Kd*derivative);
    lastError=error;
  return turn;
}

void motorMixing(){
  //code for motor mixing
  float turnError = pid();
  float turn = 0;
  float leftSpeed = baseSpeed + turnError + turn;
  float rightSpeed = baseSpeed - turnError - turn;
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  driveMotor(leftSpeed,rightSpeed);
}

void driveMotor(float left, float right) {
  // --- LEFT MOTOR ---
  analogWrite(enable_1, right); 
  digitalWrite(out_1, HIGH);   // Setting these to HIGH/LOW 
  digitalWrite(out_2, LOW);    // makes the motor go FORWARD

  // --- RIGHT MOTOR ---
  analogWrite(enable_2, left);
  digitalWrite(out_3, HIGH);   // Setting these to HIGH/LOW
  digitalWrite(out_4, LOW);    // makes the motor go FORWARD
}
float turn_angle(){
  if(analogRead(ir_1)==maxir_1 && analogRead(ir_2)==maxir_2)
  {
    return 10;
  }else if(analogRead(ir_1)==maxir_1 && analogRead(ir_3)==maxir_3)
  {return -10;}
  else{return 0;}
}
void loop() {
  // to run only and only if slider 2 is high
  if(digitalRead(slider_2)==HIGH)
  {
    motorMixing();
  }
  else{
    //indicator to reset bot
    stop_motor();
    digitalWrite(led,LOW);
  }
}
