#include <Arduino.h>
#include <FiniteStateMachine.h>

#define MAX_SPEED 255

#define MIN_ATTACK_DISTANCE 400 // test values
#define MIN_DETECT_DISTANCE 300 // test values

#define MARGIN_TRESHOLD 100 //NO idea what real values should be here

#define ROTATE_LEFT 0
#define ROTATE_RIGHT 1

#define DISTANCE_PIN 0
#define MOTOR2_PIN1  3
#define MOTOR2_PIN2  5
#define MOTOR1_PIN1  6
#define MOTOR1_PIN2  9

int lastDistance = -1;
int lastRotate = -1; 

// Global sensor data

int distance;
int leftColor;
int rightColor;
int leftLight;
int rightLight;

// Sumo methods forward-declaration
void startMoving(int speedLeft, int speedRight);

// FSM Globals

//'callback' is function pointer type, to a void function with one pointer parameter (void func(void*))
typedef void (*callback)(void *);

// when we expect the state to change
unsigned long targetTime = 0;
//what to do when state change should occur
callback pDeferredMethod = NULL;
void *pData = NULL;

// FSM tools Forward-declaration
void startFindOpponentLeft();
void startFindOpponentRight();
void onFindOponent();

void startCloseIn();
void onClosingIn();

void startAttack();
void onAttacking();

void goBack();

// State consumption method - just see if we need to move to our new state
void checkTimer(); 
// Standard deferred action - move to a predetermined state
void timedTransition(void *pData);

void cleanupTimer() {
  targetTime = 0;
  pDeferredMethod = NULL;
  pData = NULL;

  //don't forget to stop the motors
  startMoving(0, 0);
}

//initialize states & FSM
State FindOpponentLeft = State(startFindOpponentLeft, onFindOponent, cleanupTimer);
State FindOpponentRight = State(startFindOpponentRight, onFindOponent, cleanupTimer);
State CloseIn = State(startCloseIn, onClosingIn, cleanupTimer);
State Attack = State(startAttack, onAttacking, cleanupTimer);
State GoBack = State(goBack, checkTimer, cleanupTimer);

FSM sumoStateMachine = FSM(FindOpponentLeft);     //initialize state machine, start in state: FindOpponentLeft

// FSM state implementation

void scheduleMethodCall(unsigned int timeout, callback method, void *pInitData) {
  targetTime = millis() + timeout;

  pDeferredMethod = method;
  pData = pInitData;
}

void startFindOpponentRight() {
}

void startFindOpponentLeft() {
  //Rotate left
  startMoving(-MAX_SPEED, MAX_SPEED);

  //TODO: check if changing to the same state works!
  scheduleMethodCall(100, timedTransition, &FindOpponentLeft);
}

void onFindOponent() {
  if (distance <= MIN_DETECT_DISTANCE) {
    //found the enemy!
    sumoStateMachine.immediateTransitionTo(CloseIn);
    return;
  }
  checkTimer(); //don't forget to call this!
}

void startCloseIn() {
  startMoving(MAX_SPEED, MAX_SPEED);

  //unless something interrupts us, go back to the 'find oponent' states
  scheduleMethodCall(100, timedTransition, &FindOpponentLeft);
}

void onClosingIn() {
  if (distance <= MIN_ATTACK_DISTANCE) {
    //found the enemy!
    sumoStateMachine.immediateTransitionTo(Attack);
    return;
  }
  if (distance > MIN_DETECT_DISTANCE) {
    //completely lost the enemy -> start searching again
    sumoStateMachine.immediateTransitionTo(FindOpponentLeft);
    return;
  }
  checkTimer(); //don't forget to call this!
}

void startAttack() {
  startMoving(MAX_SPEED, MAX_SPEED);

  //Ram him for max. 2 second. Then try to find him again
  scheduleMethodCall(2000, timedTransition, &FindOpponentLeft);
}

void onAttacking() {
  if (distance > MIN_DETECT_DISTANCE) {
    //completely lost the enemy -> start searching again
    sumoStateMachine.immediateTransitionTo(FindOpponentLeft);
    return;
  }
  if (distance > MIN_ATTACK_DISTANCE) {
    //Oponent ran away, but still ahead. Not much to do..
  }
  checkTimer(); //don't forget to call this!

}

void goBack() {
  startMoving(-MAX_SPEED, -MAX_SPEED);

  //Go back for one full second
  scheduleMethodCall(1000, timedTransition, &FindOpponentLeft);
}

void checkTimer() {
  unsigned long now = millis();
  if (targetTime > 0 && now >= targetTime) {    
    if (pDeferredMethod) {
      pDeferredMethod(pData);
    } else {
      Serial.println("No target state");
    }
  }
}

// method is of type 'callback'
void timedTransition(void *pData) {
  if (pData) {
    Serial.println("Performing transition");
    sumoStateMachine.immediateTransitionTo(*((State *)pData));
  } else {
    Serial.println("No data for timed transition");
  }
}


void setup(){ 
  Serial.begin(115200);
  
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
}

void loop(){
  //TODO: Read all sensors, not just distance
  distance = analogRead(DISTANCE_PIN);

  //global condition, will break out of any state
  if (leftColor >= MARGIN_TRESHOLD || rightColor >= MARGIN_TRESHOLD) {
    //action will really be executed in the .update() call below
    sumoStateMachine.transitionTo(GoBack);
  }

  sumoStateMachine.update();
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
