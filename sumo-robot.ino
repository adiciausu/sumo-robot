#define MAX_SPEED 255

//#define MIN_ATTACK_DISTANCE 200 // inverse proportion
//#define MIN_DETECT_DISTANCE 80 // inverse proportion

#define MIN_ATTACK_DISTANCE 400 // test values
#define MIN_DETECT_DISTANCE 300 // test values

#define ROTATE_LEFT 0
#define ROTATE_RIGHT 1

#define DISTANCE_PIN 0
#define MOTOR2_PIN1  3
#define MOTOR2_PIN2  5
#define MOTOR1_PIN1  6
#define MOTOR1_PIN2  9

int lastDistance = -1;
int lastRotate = -1; 

void setup() {
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // @todo check if sensors see border and do something or continue to detectAndAttack();
  detectAndAttack();
}


void detectAndAttack(){
  int distance = analogRead(DISTANCE_PIN);
  Serial.println(distance);
  
  if (distance > MIN_ATTACK_DISTANCE) { // attack if close
    Serial.println("Attack!");
    startMovingForward();
    return;
  }
  
  if (distance > MIN_DETECT_DISTANCE) { // move closer if you can see him
    Serial.println("Move closer!");
    moveForward(100); // move closer
  } else {
    Serial.println("Rotate to find!");
    rotateLeft(100); // rotate to find opponent
  }
}


void detectAndAttackv2Untested(){
  int distance = analogRead(DISTANCE_PIN);
  Serial.println(distance);
  
  if (distance > MIN_ATTACK_DISTANCE) { // attack if close
    Serial.println("Attack!");
    startMovingForward();
    return;
  }
  
  if (distance > MIN_DETECT_DISTANCE) { // move closer if you can see him
    Serial.println("Move closer!");
    moveForward(100); // move closer
  } else {
    Serial.println("Rotate to find opponent!");
    if (lastDistance > MIN_DETECT_DISTANCE && lastRotate == ROTATE_LEFT) {
      rotateRight(200); // twice to compensate previous rotate left
    } else {
      rotateLeft(100);  
    }
  }
  
  lastDistance = distance;
}

void moveForward(int time) {
    startMovingForward();
    delay(time);
    stopMoving();
}
 
void rotateLeft(int time) {
  startMoving(0, MAX_SPEED);
  delay(time);
  stopMoving();
  lastRotate = ROTATE_LEFT;
}

void rotateRight(int time) {
  startMoving(MAX_SPEED, 0);
  delay(time);
  stopMoving();
  lastRotate = ROTATE_RIGHT;
}

void startMovingForward() {
  startMoving(MAX_SPEED, MAX_SPEED);
}

void stopMoving() {
  startMoving(0, 0);
}

void startMoving(int speedLeft, int speedRight) {
  if (speedLeft > 0) {
    analogWrite(MOTOR1_PIN1, speedLeft);
    analogWrite(MOTOR1_PIN2, 0);
  }  else {
    analogWrite(MOTOR1_PIN1, 0);
    analogWrite(MOTOR1_PIN2, -speedLeft);
  }
 
  if (speedRight > 0) {
    analogWrite(MOTOR2_PIN1, speedRight);
    analogWrite(MOTOR2_PIN2, 0);
  } else {
    analogWrite(MOTOR2_PIN1, 0);
    analogWrite(MOTOR2_PIN2, -speedRight);
  }
}
