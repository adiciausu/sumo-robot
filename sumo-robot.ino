#include <Arduino.h>
#include <FiniteStateMachine.h>
#include <QTRSensors.h>

/*
 *    DEBUGGING helpers. Uncomment to enable their behavior
 */

#define WITH_DO_NOTHING
//#define NO_QUICK_180
//#define NO_WAIT_AT_START
//#define NO_SWITCH_DIRECTION
#define NO_GOBACK
//#define NO_MOVING

//#define ONLY_ROTATE

//#define LOG_SENSOR_READINGS


#define MAX_SPEED 255
#define ROTATE_SPEED 160
#define CLOSEIN_SPEED 150

#define MIN_ATTACK_DISTANCE 300 // inverse proportion
#define MIN_DETECT_DISTANCE 170 // inverse proportion

#define MARGIN_TRESHOLD_L 200 //80 (only white edge)
#define MARGIN_TRESHOLD_R 750 //300 (only white edge)

#define DISTANCE_PIN 0
#define MOTOR2_PIN1  3
#define MOTOR2_PIN2  5
#define MOTOR1_PIN1  6
#define MOTOR1_PIN2  9

#define NUM_SENSORS   2     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low

// sensors are connected to pin 7 digital and 8 digital
QTRSensorsRC qtrrc((unsigned char[]) {
  7, 8
},
NUM_SENSORS, TIMEOUT);

int SpinDirection;

// Global sensor data

int distance;
unsigned int sensorValues[NUM_SENSORS];
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
void startQuick180();

void startFindOpponent();
void onFindOponent();

void startCloseIn();
void onClosingIn();

void startAttack();
void onAttacking();

void goBack();

void doNothing();

// State consumption method - just see if we need to move to our new state
void checkTimer(); 
// Standard deferred action - move to a predetermined state
void timedTransition(void *pData);

void onClosingInExpired(void *pData);

void cleanupTimer() {
  targetTime = 0;
  pDeferredMethod = NULL;
  pData = NULL;

  //Motors don't usually need to be stopped. The next state will issue a new command for them anyway
  //startMoving(0, 0);
}

//initialize states & FSM
State Quick180 = State(startQuick180, onFindOponent, cleanupTimer);
State FindOpponent = State(startFindOpponent, onFindOponent, cleanupTimer);
State CloseIn = State(startCloseIn, onClosingIn, cleanupTimer);
State Attack = State(startAttack, onAttacking, cleanupTimer);
State GoBack = State(goBack, checkTimer, cleanupTimer);
State DoNothing = State(doNothing);

FSM sumoStateMachine = FSM(Quick180);  //initialize state machine, start in state: FindOpponent

// FSM state implementation

void scheduleMethodCall(unsigned int timeout, callback method, void *pInitData) {
  targetTime = millis() + timeout;

  pDeferredMethod = method;
  pData = pInitData;
}

void startQuick180() {
#ifdef NO_QUICK_180
  sumoStateMachine.transitionTo(FindOpponent);
  return;
#endif

#define QUICK_ROTATE_SPEED 255
#define QUICK_ROTATE_TIME 600

  Serial.println("State - Quick180");
  startMoving(-QUICK_ROTATE_SPEED, QUICK_ROTATE_SPEED);
  scheduleMethodCall(QUICK_ROTATE_TIME, timedTransition, &FindOpponent);
  //scheduleMethodCall(QUICK_ROTATE_TIME, timedTransition, &DoNothing);
}

//callback to reverse the rotation direction
void reverseRotation(void*);

//common implementation for both State init and reverRotation above
void startSpinning() {
   startMoving(-ROTATE_SPEED * SpinDirection, ROTATE_SPEED * SpinDirection);
  
  //Move differently after a whole rotation
  //TODO: figure out how long it takes to do a full rotation

#ifndef NO_SWITCH_DIRECTION  
  scheduleMethodCall(2500, reverseRotation, &FindOpponent);
#endif
}

void reverseRotation(void*) {
  
  SpinDirection = -SpinDirection;
  if (SpinDirection > 0) {
    Serial.println("Reverse spinning - LEFT spin");
  } else {
    Serial.println("Reverse spinning - Right spin");
  }

  startSpinning();  
}

void startFindOpponent() {
  Serial.println("State - FindOpponent");
  
  //Global variable
  SpinDirection = 1; //default - turn to the left (counterclockwise)
  if (random(0, 2) == 0) {
    Serial.println("start find opponent Right state");
    SpinDirection = -1; //right/clockwise
  } else {
    Serial.println("start find opponent Left state");
  }

  startSpinning(); 
}

void onFindOponent() {
  if (distance >= MIN_DETECT_DISTANCE) {
    //found the enemy!
#ifndef ONLY_ROTATE    
    sumoStateMachine.immediateTransitionTo(CloseIn);
    return;
#endif
  }
  checkTimer(); //don't forget to call this!
}

void startCloseIn() {
  Serial.println("State - ClosingIn");
  startMoving(CLOSEIN_SPEED, CLOSEIN_SPEED);

  //unless something interrupts us, go back to the 'find oponent' states
  scheduleMethodCall(500, onClosingInExpired, &FindOpponent);
}

void onClosingIn() {
  if (distance >= MIN_ATTACK_DISTANCE) {
    //found the enemy!
#ifndef ONLY_ROTATE
    sumoStateMachine.immediateTransitionTo(Attack);
    return;
#endif
  }
  /*
  if (distance < MIN_DETECT_DISTANCE) {
    //completely lost the enemy -> start searching again
    Serial.println("Lost enemy");
    sumoStateMachine.immediateTransitionTo(FindOpponent);
    return;
  }
  */
  checkTimer(); //don't forget to call this!
}

void onClosingInExpired(void *pData) {
  if (distance < MIN_DETECT_DISTANCE) {
    Serial.println("Lost enemy");
    sumoStateMachine.immediateTransitionTo(FindOpponent);
  } else {
    Serial.println("Continue with closein");
    sumoStateMachine.immediateTransitionTo(CloseIn);
  }
}

void startAttack() {
  Serial.println("State - Attack");
  startMoving(MAX_SPEED, MAX_SPEED);

  //Ram him for max. 2 second. Then try to find him again
  scheduleMethodCall(2000, timedTransition, &FindOpponent);
}

void onAttacking() {
  if (distance < MIN_DETECT_DISTANCE) {
    //completely lost the enemy -> start searching again
    Serial.println("Attack - lost enemy");
    sumoStateMachine.immediateTransitionTo(FindOpponent);
    return;
  }
  if (distance < MIN_ATTACK_DISTANCE) {
    Serial.println("No longer in attack range");
    //Oponent ran away, but still ahead. Not much to do..
  }
  checkTimer(); //don't forget to call this!

}

void goBack() {
  Serial.println("State - GoBack");
  int speedLeft = MAX_SPEED/2;
  int speedRight = MAX_SPEED/2;
  if(sensorValues[0] <= MARGIN_TRESHOLD_L) {
    speedRight = MAX_SPEED;
  }
  if(sensorValues[1] <= MARGIN_TRESHOLD_R) {
    speedLeft = MAX_SPEED;
  }
  
  startMoving(-speedLeft, -speedRight);
    
  scheduleMethodCall(500, timedTransition, &FindOpponent);
}

void doNothing() {
  Serial.println("State - DoNothing");
 #ifdef WITH_DO_NOTHING
  startMoving(0, 0);
  cleanupTimer();
 #else
  sumoStateMachine.immediateTransitionTo(FindOponent);
 #endif
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
    //Serial.println("Performing transition");
    sumoStateMachine.immediateTransitionTo(*((State *)pData));
  } else {
    Serial.println("No data for timed transition");
  }
}


void setup(){ 
  Serial.begin(9600);
  
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);

  //random number generator init
  randomSeed(analogRead(0));
  
#ifndef NO_WAIT_AT_START
  Serial.println("Starting countdown");
  delay(5000);
#endif

  Serial.println("Start!");
}

void loop(){
  //TODO: Also read light sensors
  distance = analogRead(DISTANCE_PIN);
  qtrrc.read(sensorValues);

#ifdef LOG_SENSOR_READINGS
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("; Color sensors: ");
  Serial.print(sensorValues[0]);
  Serial.print(", ");
  Serial.print(sensorValues[1]);
  
  Serial.println("");
#endif
  
#ifndef NO_GOBACK
  //global condition, will break out of any state
  if (sensorValues[0] <= MARGIN_TRESHOLD_L || sensorValues[1] <= MARGIN_TRESHOLD_R) {
    //action will really be executed in the .update() call below
    sumoStateMachine.transitionTo(GoBack);
  }
#endif

  sumoStateMachine.update();
}


void startMoving(int speedLeft, int speedRight) {
#ifdef NO_MOVING
  return;
 #endif
  
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
