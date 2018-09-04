#include <Arduino.h>
#include <FiniteStateMachine.h>
#include <QTRSensors.h>
#include <Average.h>  

/*
 *    DEBUGGING helpers. Uncomment to enable their behavior
 */

//#define WITH_DO_NOTHING
//#define NO_WAIT_AT_START
//#define NO_SWITCH_DIRECTION

#define NO_QUICK_180
//#define NO_GOBACK
#define NO_FORFEIT
//#define NO_MOVING
//#define NO_SWEEP
//#define NO_LIGHT_SENSORS

#define USE_SMOOTHING

#define LOG_SENSOR_READINGS

#define MAX_SPEED 255
#define ROTATE_SPEED 160
#define CLOSEIN_SPEED 150

#define QUICK_ROTATE_SPEED 255
#define QUICK_ROTATE_TIME 800
#define TURN_180_LIGHT_TIME 1200

#define SWEEP_SPEED 180
#define SWEEP_TIME 300

#define FIGHT_TIME_MILIS 57000 //3 seconds to spare to exit the ring

#define MIN_ATTACK_DISTANCE 300 // inverse proportion
#define MIN_DETECT_DISTANCE 170 // inverse proportion

#define MARGIN_TRESHOLD_L 300 //80 (only white edge)
#define MARGIN_TRESHOLD_R 300 //300 (only white edge)

#define LIGHT_LIMIT_L 80
#define LIGHT_LIMIT_R 80

#define DISTANCE_PIN 0
#define LIGHT_LEFT_SENSOR 1
#define LIGHT_RIGHT_SENSOR 2
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

int SpinDirection = 1;
int startedAtMiliseconds;

// Global sensor data

int distance;
Average<int> distanceSeries(5);

unsigned int sensorValues[NUM_SENSORS];
int leftLight;
int rightLight;


#define DISTANCE_SMOOTH_FACTOR 0.75
#define LIGHT_SMOOTH_FACTOR 0.5
int smooth(int data, float filterVal, float smoothedVal);

// Sumo methods forward-declaration
void startMoving(int speedLeft, int speedRight);

// LEDs
#define LED_YELLOW 4
#define LED_BLUE 10
#define LED_RED 11

// FSM Globals

//'callback' is function pointer type, to a void function with one pointer parameter (void func(void*))
typedef void (*callback)(void *);

// when we expect the state to change
unsigned long targetTime = 0;
//what to do when state change should occur
callback pDeferredMethod = NULL;
void *pData = NULL;

// FSM tools Forward-declaration

// Oponent finding states
void startQuick180();
void onQuick180();

void startFindOpponent();
void onFindOponent();

void startSweep();

// Attack & attack preparation states
void startCloseIn();
void onClosingIn();

void startAttack();
void onAttacking();

// Boundaries & special cases
void goBack();
void doNothing();
void forfeit();

// Light-sensor states
void startRamBackwards();
void onRamBackwards();
void start180Turn();
void on180Turn();

bool willSwitchToAttackState();

// State consumption method - just see if we need to move to our new state
void checkTimer(); 
// Standard deferred action - move to a predetermined state
void timedTransition(void *pData);

void onClosingInExpired(void *pData);

int getBlinkState() { return millis() % 100 < 50 ? HIGH : LOW; }
void clearLEDs() {
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
}

void cleanupTimer() {
  targetTime = 0;
  pDeferredMethod = NULL;
  pData = NULL;

  clearLEDs();
  //Motors don't usually need to be stopped. The next state will issue a new command for them anyway
  //startMoving(0, 0);
}

//initialize states & FSM
State Quick180 = State(startQuick180, onQuick180, cleanupTimer);
State FindOpponent = State(startFindOpponent, onFindOponent, cleanupTimer);
State SweepLeftRight = State(startSweep, onFindOponent, cleanupTimer);

State CloseIn = State(startCloseIn, onClosingIn, cleanupTimer);
State Attack = State(startAttack, onAttacking, cleanupTimer);

State GoBack = State(goBack, checkTimer, cleanupTimer);
State DoNothing = State(doNothing);
State Forfeit = State(forfeit);

State RamBackwards = State(startRamBackwards, onRamBackwards, cleanupTimer);
State TurnOnBackLight = State(start180Turn, on180Turn, cleanupTimer);

#ifdef NO_QUICK_180 
  #define START_STATE FindOpponent
#else
  #define START_STATE Quick180
#endif

FSM sumoStateMachine = FSM(START_STATE);

#ifdef NO_SWEEP 
  #define LOCK_LOST_STATE FindOpponent
#else 
  #define LOCK_LOST_STATE SweepLeftRight
#endif


// FSM state implementation

void scheduleMethodCall(unsigned int timeout, callback method, void *pInitData) {
  targetTime = millis() + timeout;

  pDeferredMethod = method;
  pData = pInitData;
}

void startQuick180() {
  Serial.println("State - Quick180");
  digitalWrite(LED_BLUE, HIGH); //actually will be a blink action
  
  startMoving(-QUICK_ROTATE_SPEED, QUICK_ROTATE_SPEED);
  //Maybe always switch to FindOponent since we don't really know if he's here or not?
  scheduleMethodCall(QUICK_ROTATE_TIME, timedTransition, &LOCK_LOST_STATE); 
}

//Just needed to blink the blue led
void onQuick180() {
  digitalWrite(LED_BLUE, getBlinkState());
  
  onFindOponent();
}

//callback to reverse the rotation direction
void reverseRotation(void*);

//common implementation for both State init and reverRotation above
void startSpinning() {
   startMoving(-ROTATE_SPEED * SpinDirection, ROTATE_SPEED * SpinDirection);
  
  //Move differently after a whole rotation
  //TODO: figure out how long it takes to do a full rotation

#ifndef NO_SWITCH_DIRECTION  
  scheduleMethodCall(2500, reverseRotation, NULL);
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
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  
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

void reverseSweep(void *) {

  SpinDirection = -SpinDirection;
  Serial.println("Reverse Sweep - LEFT");
  
  startMoving(-SWEEP_SPEED * SpinDirection, SWEEP_SPEED * SpinDirection);
  //wait for twice as long so we don't just get back to the origin
  scheduleMethodCall(2*SWEEP_TIME, timedTransition, &FindOpponent);
}

void startSweep() {
  Serial.println("State - Start sweep");
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  //start oposite to whatever we were doing before this. Not sure it's helpful
  SpinDirection = -SpinDirection;

  startMoving(-SWEEP_SPEED * SpinDirection, SWEEP_SPEED * SpinDirection);

  scheduleMethodCall(SWEEP_TIME, reverseSweep, NULL);
}

bool willSwitchToAttackState() {
    if (distance >= MIN_ATTACK_DISTANCE) {
    //found the enemy!
    sumoStateMachine.immediateTransitionTo(Attack);
    return true;
  }
  if (distance >= MIN_DETECT_DISTANCE) {
    //found the enemy!
    sumoStateMachine.immediateTransitionTo(CloseIn);
    return true;
  }

  return false;

}

bool willSwitchBasedOnLightLevels() {
#ifdef NO_LIGHT_SENSORS
  return false;
#endif

  if (leftLight < LIGHT_LIMIT_L && rightLight < LIGHT_LIMIT_R) {
    Serial.println("Oponent at back - Ram backward");
    sumoStateMachine.immediateTransitionTo(RamBackwards);
    return true;
  }

  //check each direction - but only if we're not spinning that way already
  if (leftLight < LIGHT_LIMIT_L && !sumoStateMachine.isInState(TurnOnBackLight) && SpinDirection != 1) {
    Serial.println("Oponent at back Left - Turn right");
    SpinDirection = 1;
    sumoStateMachine.immediateTransitionTo(TurnOnBackLight);
    return true;
  }
  if (rightLight < LIGHT_LIMIT_R && !sumoStateMachine.isInState(TurnOnBackLight) && SpinDirection != -1) {
    Serial.println("Oponent at back Right - Turn left");
    SpinDirection = -1;
    sumoStateMachine.immediateTransitionTo(TurnOnBackLight);
    return true;
  }

  return false;
}

void onFindOponent() {

  if (willSwitchToAttackState()) {
    return;
  }

  if (willSwitchBasedOnLightLevels()) {
    return;
  }
  
  checkTimer(); //don't forget to call this!
}

void startRamBackwards() {
  Serial.println("State - RamBackwards");
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  startMoving(-MAX_SPEED, -MAX_SPEED);
}

void onRamBackwards() {
  int blinkSignal = getBlinkState();
  
  digitalWrite(LED_RED, blinkSignal);
  digitalWrite(LED_BLUE, blinkSignal);
  
  if (willSwitchToAttackState()) {
    return;
  }

  //Check if we should still be pushing backwards
  if (leftLight > LIGHT_LIMIT_L && rightLight > LIGHT_LIMIT_R) {
    Serial.println("Ram backward - lost completely");
    sumoStateMachine.immediateTransitionTo(Quick180);
    return true;
  }
  //only one sensor might lose it
  if (leftLight > LIGHT_LIMIT_L) {
    Serial.println("Ram backward - lost on Left -> turn right");
    SpinDirection = 1;
    sumoStateMachine.immediateTransitionTo(TurnOnBackLight);
    return true;    
  }
  if (rightLight > LIGHT_LIMIT_R) {
    Serial.println("Ram backward - lost on Right -> turn Left");
    SpinDirection = -1;
    sumoStateMachine.immediateTransitionTo(TurnOnBackLight);
    return true;    
  }

  checkTimer(); //don't forget to call this!
}


void start180Turn() {
  Serial.println("State - TurnOnBackLight");
  //actually one of the LEDs will blink
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  startMoving(SpinDirection > 0 ? MAX_SPEED : 0, SpinDirection > 0 ? 0: MAX_SPEED);
  
  //TODO: figure out how long it takes to do a full rotation
  scheduleMethodCall(TURN_180_LIGHT_TIME, timedTransition, &LOCK_LOST_STATE);
}

//only needed for the blinking
void on180Turn() {
  if (SpinDirection > 0) {
    digitalWrite(LED_RED, getBlinkState());
  } else {
    digitalWrite(LED_BLUE, getBlinkState());
  }
  onFindOponent();
}

void startCloseIn() {
  Serial.println("State - ClosingIn");
  digitalWrite(LED_BLUE, HIGH);
  
  startMoving(CLOSEIN_SPEED, CLOSEIN_SPEED);

  //unless something interrupts us, go back to the 'find oponent' states
  scheduleMethodCall(1000, onClosingInExpired, &LOCK_LOST_STATE);
}

void onClosingIn() {
  if (distance >= MIN_ATTACK_DISTANCE) {
    //found the enemy!
    sumoStateMachine.immediateTransitionTo(Attack);
    return;
  }
  //TODO: why is this being triggered?
  /*
  if (distance < MIN_DETECT_DISTANCE) {
    //completely lost the enemy -> start searching again
    Serial.println("Lost enemy");
    sumoStateMachine.immediateTransitionTo(LOCK_LOST_STATE);
    return;
  }
  */
  checkTimer(); //don't forget to call this!
}

void onClosingInExpired(void*) {
  if (distance < MIN_DETECT_DISTANCE) {
    Serial.println("Lost enemy");
    sumoStateMachine.immediateTransitionTo(LOCK_LOST_STATE);
  } else {
    Serial.println("Continue with closein");
    sumoStateMachine.immediateTransitionTo(CloseIn);
  }
}

void startAttack() {
  Serial.println("State - Attack");
  digitalWrite(LED_RED, HIGH);
  
  startMoving(MAX_SPEED, MAX_SPEED);

  //Ram him for max. 2 second. Then try to find him again
  // OR, just keep on going while we're still locked on
  //Another option would be to back off a bit, then ram again. Maybe the added impulse would buldge it more
  //scheduleMethodCall(2000, timedTransition, &LOCK_LOST_STATE);
}

void onAttacking() {
  if (distance < MIN_DETECT_DISTANCE) {
    //completely lost the enemy -> start searching again
    Serial.println("Attack - lost enemy");
    sumoStateMachine.immediateTransitionTo(LOCK_LOST_STATE);
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
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);
  
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

void forfeit() {
  Serial.println("State - Forfeit game");
  
  int blink = getBlinkState();
  digitalWrite(LED_YELLOW, blink);
  digitalWrite(LED_BLUE, blink);
  digitalWrite(LED_RED, blink);

  startMoving(MAX_SPEED, MAX_SPEED);
}

void doNothing() {
  Serial.println("State - DoNothing");
 #ifdef WITH_DO_NOTHING
  startMoving(0, 0);
  cleanupTimer();
 #else
  sumoStateMachine.immediateTransitionTo(FindOpponent);
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

  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  
  //random number generator init
  randomSeed(analogRead(0));

  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  startedAtMiliseconds = millis();
  
#ifndef NO_WAIT_AT_START
  Serial.println("Starting countdown");
  delay(5000);
#endif

  Serial.println("Start!");

  clearLEDs();
}

void loop(){
  // From tests: 6ms between loop execution
  //Serial.println(millis());

  //Read all sensors
  int rawDistance = analogRead(DISTANCE_PIN);
  int rawLeftLight = analogRead(LIGHT_LEFT_SENSOR);
  int rawRightLight = analogRead(LIGHT_RIGHT_SENSOR);

#ifdef USE_SMOOTHING
  distance = readDistance();
//distance = smooth(distance, DISTANCE_SMOOTH_FACTOR, rawDistance);
  leftLight = smooth(leftLight, LIGHT_SMOOTH_FACTOR, rawLeftLight);
  rightLight = smooth(rightLight, LIGHT_SMOOTH_FACTOR, rawRightLight);
#else
  distance = rawDistance;
  leftLight = rawLeftLight;
  rightLight = rawRightLight;
#endif

  qtrrc.read(sensorValues);

#ifdef LOG_SENSOR_READINGS
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("; Color sensors: ");
  Serial.print(sensorValues[0]);
  Serial.print(", ");
  Serial.print(sensorValues[1]);
  Serial.print("; Light:  ");
  Serial.print(leftLight);
  Serial.print(", ");
  Serial.print(rightLight);
  Serial.println("");
#endif
  
#ifndef NO_GOBACK
  //global condition, will break out of any state
  if (sensorValues[0] <= MARGIN_TRESHOLD_L || sensorValues[1] <= MARGIN_TRESHOLD_R) {
    //action will really be executed in the .update() call below
    sumoStateMachine.transitionTo(GoBack);
  }
#endif

#ifndef NO_FORFEIT
  //global condition, will break out of any state
  if (startedAtMiliseconds + FIGHT_TIME_MILIS < millis()) {
    //action will really be executed in the .update() call below.
    // This should override previous transition
    sumoStateMachine.transitionTo(Forfeit);
  }
#endif

  sumoStateMachine.update();
}

int readDistance() {
  int currentDistance = analogRead(DISTANCE_PIN);
  int meanDistance = distanceSeries.mean();
  distanceSeries.push(currentDistance); // self capped

  if (abs(meanDistance - currentDistance) > 0.3 * meanDistance) {
      Serial.println(" ");
      Serial.print(currentDistance);
      Serial.print(" >>> ");
      Serial.print(meanDistance);
      Serial.println(" ");
      
      return meanDistance;
  }
  
  return currentDistance;
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

int smooth(int data, float filterVal, float smoothedVal) {
/*
  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }
*/
  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}
