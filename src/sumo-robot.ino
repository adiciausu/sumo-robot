#include <FiniteStateMachine.h>


//#define LED_RED 10
//#define LED_GREEN 11
#define LED_BLUE 9

//'callback' is function pointer type, to a void function with one pointer parameter (void func(void*))
typedef void (*callback)(void *);

// when we expect the state to change
unsigned long targetTime = 0;
//what to do when state change should occur
callback pDeferredMethod = NULL;
void *pData = NULL;

// Forward-declaration
void startLedOn();
void startLedOff();

// State consumption method - just see if we need to move to our new state
void checkTimer(); 
// Standard deferred action - move to a predetermined state
void timedTransition(void *pData);

//utility functions
void scheduleMethodCall(unsigned int timeout, callback method, void *pInitData) {
  targetTime = millis() + timeout;

  pDeferredMethod = method;
  pData = pInitData;
}

void cleanupTimer() {
 Serial.println("Cleanup");
 targetTime = 0;
 pDeferredMethod = NULL;
 pData = NULL;
}

//end utility functions 

//initialize states & FSM
State On = State(startLedOn, checkTimer, cleanupTimer);
State Off = State(startLedOff, checkTimer, cleanupTimer);
 
FSM ledStateMachine = FSM(On);     //initialize state machine, start in state: On

// FSM state implementation

void startLedOn() {
  Serial.println("State ON");
  digitalWrite(LED_BLUE, HIGH);

  scheduleMethodCall(1000, timedTransition, &Off);
}
void startLedOff() {
  Serial.println("State OFF");
  digitalWrite(LED_BLUE, LOW);

  scheduleMethodCall(2000, timedTransition, &On);
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
    ledStateMachine.immediateTransitionTo(*((State *)pData));
  } else {
    Serial.println("No data for timed transition");
  }
}


void setup(){ 
  Serial.begin(115200);
  //while (!Serial);

  //pinMode(LED_RED, OUTPUT);
  //pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  ledStateMachine.transitionTo(On);
}

void loop(){
  ledStateMachine.update();
}
