#include "MeMCore.h"
#include "stdbool.h"

//BUZZER
MeBuzzer buzzer;
//char i;
int note, duration;

//ULTRASONIC SENSOR
MeUltrasonicSensor ultrasonic(PORT_2);
MeLineFollower lineFinder (PORT_1);
MeDCMotor motor1(M1);
MeDCMotor motor2(M2);

//Light Intensity Sensor
#define DISTANCE_LIGHT_LOWER_BOUND 800.0f
#define DISTANCE_LIGHT_UPPER_BOUND 880.0f
#define DISTANCE_LIGHT_ONE_SQUARE 700.0f


//MOTOR 2 IS THE RIGHT WHEEL, (assuming the car is facing forwards and youre looking at it from the back
bool stop_wheel1;
bool stop_wheel2;

size_t motorSpeed1 = 180;
size_t motorSpeed2 = 200;

//calculated optimal value = 8 (nominal without correction from the sensor, this is calculated using the "body" of the robot
#define DISTANCE_LOWER_BOUND 14.0f
#define DISTANCE_UPPER_BOUND 17.0f
#define DISTANCE_ONE_SQUARE 26.0f
#define RGBWait 100 //in milliseconds 

// Define time delay before taking another LDR reading
#define LDRWait 30 //in milliseconds 

#define LDR 2   //LDR sensor pin at A0
#define IR 3

// Define colour sensor LED pins
//DECODER uses analog pins 0 and 1
int B = A0;
int A = A1;

//AnalogRead will use analogpins 2 and 3


//placeholders for colour detected
int red = 0;
int green = 0;
int blue = 0;

//floats to hold colour arrays
float colourArray[] = {0, 0, 0};
float blackArray[] = {577, 487, 534}; //modify
//float whiteArray[] = {920,677,908}; //modify
float greyDiff[] = {362, 416, 385}; //modify

char colourStr[3][5] = {"R = ", "G = ", "B = "};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  analogWrite(B, 0);
  analogWrite(A, 0);
}

void infrared_linear_check()
{
  float average = 0;
  float sensor_val = analogRead(IR);
  for (int i = 0; i  < 100; i += 1)
  {
    average = average + analogRead(IR);
  }
  average /= 100;
  Serial.println(average);

  while (sensor_val < DISTANCE_LIGHT_LOWER_BOUND)
  {
    //too close to the right wall
    Serial.println(sensor_val);
    motor1.run(-200 * 0.7);
    motor2.run(185);
    //change speed, slow down right wheel
    sensor_val = analogRead(IR);
  }
}

void loop() {



  Serial.print("distance(cm) = ");
  Serial.println(ultrasonic.distanceCm());

  //infrared check
  // LIGHT INTENSITY SENSOR
  float average = 0;
  float sensor_val = analogRead(IR);

  if (sensor_val > DISTANCE_LIGHT_LOWER_BOUND) {
    // going straight
    motor1.run(-100);
    motor2.run(100);
  }
  else if (sensor_val > DISTANCE_LIGHT_ONE_SQUARE) {
    // if there is no wall, just go straight
    motor1.run(-100);
    motor2.run(100);
  }
  else if (sensor_val < DISTANCE_LIGHT_LOWER_BOUND)
  {

    //if too close to the wall on the right
    motor1.run(-200 * 0.7);
    motor2.run(185);
    Serial.println("L");
  }
  //ultrasonic check
  if (ultrasonic.distanceCm() > DISTANCE_LOWER_BOUND) {
    // going straight
    motor1.run(-100);
    motor2.run(100);
  }
  else if (ultrasonic.distanceCm() > DISTANCE_ONE_SQUARE) {
    // if there is no wall, just go straight
    motor1.run(-100);
    motor2.run(100);
  }
  else if (ultrasonic.distanceCm() < DISTANCE_LOWER_BOUND)
  {

    //if too close to the wall on the left, turn right
    motor1.run(-200);
    motor2.run(185 * 0.7);
    Serial.println("R");
  }

  int sensorState = lineFinder.readSensors();
  check_for_black_line(sensorState);

}

void check_for_black_line(int sensorState)
{
  switch (sensorState)
  {
    case S1_IN_S2_IN: motor1.stop(); motor2.stop(); delay(500) ;
      for (int c = 0; c <= 2; c++) {
        Serial.print(colourStr[c]);
        toggle_colour(c);
        delay(RGBWait);
        //get the average of 10 consecutive readings for the current colour and return an average
        colourArray[c] = getAvgReading(3);
        //  Serial.println(int(whiteArray[c]));
        //the average reading returned minus the lowest value divided by the maximum possible range, multiplied by 255 will give a value between 0-255, representing the value for the current reflectivity (i.e. the colour LDR is exposed to)
        colourArray[c] = (colourArray[c] - blackArray[c]) / (greyDiff[c]) * 255;
        toggle_colour(3);
        delay(RGBWait);
        Serial.println(int(colourArray[c])); //show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code

      }
      decideColor();
      delay(50);
      Serial.println("S1_IN_S2_IN");  break;
    case S1_IN_S2_OUT: Serial.println("S1_IN_S2_OUT"); break;
    case S1_OUT_S2_IN: Serial.println("S1_OUT_S2_IN"); break;
    case S1_OUT_S2_OUT: Serial.println ("S1_OUT_S2_OUT"); break;
    default: break;
  }
}

void turn(long turn_index, size_t motor_turnspeed1, size_t motor_turnspeed2)
{
  //left turn
  // RED COLOUR
  if (turn_index == 1)
  {
    motor1.run(motor_turnspeed1);
    motor2.run(motor_turnspeed2);
    delay(260);
  }
  //right turn
  //GREEN COLOUR
  if (turn_index == 2)
  {
    motor1.run(-motor_turnspeed1);
    motor2.run(-motor_turnspeed2);
    delay(310);

  }
  // U turn
  // ORANGECOLOUR
  if (turn_index == 3)
  {
    motor1.run(200);
    motor2.run(-180 * 0.1);
    delay(500);
    motor1.run(motor_turnspeed1);
    motor2.run(motor_turnspeed2);
    delay(390);
  }
  // 2 consecutive left turns
  //PURPLE COLOUR
  if (turn_index == 4)
  {
    motor1.run(motor_turnspeed1);
    motor2.run(motor_turnspeed2);
    delay(400);
    motor1.run(-2 * motor_turnspeed1);
    motor2.run(2 * motor_turnspeed2);
    delay(650);
    motor1.run(motor_turnspeed1);
    motor2.run(motor_turnspeed2);
    delay(300);
  }
  // 2 consecutive RIGHT turns
  //LIGHT BLUE COLOUR
  if (turn_index == 5)
  {
    motor1.run(-motor_turnspeed1);
    motor2.run(-motor_turnspeed2);
    delay(390);
    motor1.run(-motor_turnspeed1);
    motor2.run(motor_turnspeed2);
    delay(945);
    motor1.run(-motor_turnspeed1);
    motor2.run(-motor_turnspeed2);
    delay(290);
  }
}

void toggle_colour(int number)
{
  if (number == 0)
  {
    //READ RED,
    analogWrite(B, 255);
    analogWrite(A, 255);
    //digitalWrite(ledArray[c],LOW); //turn ON the LED, red, green or blue, one colour at a time.
  }

  if (number == 1)
  {
    // READ GREEN
    analogWrite(B, 0);
    analogWrite(A, 255);
  }

  if (number == 2)
  {
    //READ BLUE
    analogWrite(B, 255);
    analogWrite(A, 0);
  }

  if (number == 3)
  {
    //ALL OFF
    analogWrite(B, 0);
    analogWrite(A, 0);
  }
}



int getAvgReading(int times) {
  //find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++) {
    reading = analogRead(LDR);
    total = reading + total;
    delay(LDRWait);
  }
  //calculate the average and return it
  return total / times;
}

void decideColor() {//format color values
  if (colourArray[0] > colourArray[1] + colourArray[2] && colourArray[0] > 240 && colourArray[1] < 110 && colourArray[2] <  110) {
    Serial.println("red");
    turn(1, motorSpeed1, motorSpeed2);
    reset_colourArray();
  }
  else if (colourArray[0] > colourArray[1] && colourArray[1] > colourArray[2] && colourArray[0] < 300 && colourArray[0] > 190 && colourArray[1] < 160  && colourArray[2] < 130) {
    Serial.println("orange");
    turn(3, motorSpeed1, motorSpeed2);
    reset_colourArray();
  }

  else if ( colourArray[2] > colourArray[0] &&  colourArray[2] > colourArray[1] && colourArray[0] < 220 && colourArray[1] < colourArray[0] && colourArray[0] > 140 && colourArray[1] < 190 && colourArray[1] > 140 && colourArray[2] < 230) {
    Serial.println("purple");
    turn(4, motorSpeed1, motorSpeed2);
    reset_colourArray();
  }
  else if (colourArray[1] > colourArray[0] && colourArray[1] > colourArray[2] && colourArray[0] < 200 && colourArray[1] > 150 && colourArray[2] < 200 && colourArray[2] > 120) {
    Serial.println("green");
    turn(2, motorSpeed1, motorSpeed2);
    reset_colourArray();
  }
  else if ( colourArray[2] > colourArray[1] && colourArray[2] > colourArray[0] && colourArray[0] < 190 && colourArray[1] < 255 && colourArray[1] > 180 && colourArray[2] > 200)
  {
    Serial.println("blue");
    turn(5, motorSpeed1, motorSpeed2);
    reset_colourArray();
  }
  else if (colourArray[0] > 230 && colourArray[1] > 230 && colourArray[2] > 200)
  {
    Serial.println("white");
    buzzer_noise();
  }

  else {
    Serial.println("unknown");
  }
}

void reset_colourArray()
{
  colourArray[0] = 0;
  colourArray[1] = 0;
  colourArray[2] = 0;
}


void buzzer_noise()
{
  for (int i = 0; i < 50; i += 1)
  {
    note = random(100, 1500);
    duration = random(100, 200);
    buzzer.tone(note, duration);
  }
  delay(100);
}

/*
void infrared_linear_check()
{
  float average = 0;
  float sensor_val = analogRead(IR);
  for (int i = 0; i  < 100; i += 1)
  {
    average = average + analogRead(IR);
  }
  average /= 100;
  Serial.println(average);

  while (sensor_val < DISTANCE_LIGHT_LOWER_BOUND)
  {
    //too close to the right wall, turn right by slowing down the right wheel
    Serial.println(sensor_val);
    motor1.run(-200 * 0.7);
    motor2.run(185);
    //change speed, slow down right wheel
    sensor_val = analogRead(IR);
  }
}
*/
