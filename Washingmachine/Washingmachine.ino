// Sensor to detect washing machine end of cycle
// Connect vibration sensor to digital pin 3 
//https://forum.mysensors.org/topic/1617/washing-machine-ended-sensor/2

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_RF24

#define MY_NODE_ID 46 // Sets a static id for a node //22 AlarmEnabledSensor
#define MY_RF24_PA_LEVEL RF24_PA_MAX // Max tx power

//Enable Repeater function
#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>

#define CHILD_ID 3
#define BUTTON_PIN  3  // Arduino Digital I/O pin for vibration sensor
#define LED_PIN  4  // Arduino Digital I/O pin for LED indicator
#define SAMPLING_PERIOD 300000 // 5 minutes period. YMMV

int oldValue=-1;
unsigned long period_start = millis();
int state = -1;


MyMessage msg(CHILD_ID,V_TRIPPED);


void setup()  
{  
    //begin();
    Serial.println("Setup pins: ");
    // Setup the button
    pinMode(BUTTON_PIN,INPUT);
    pinMode(LED_PIN,OUTPUT);
    digitalWrite(LED_PIN,LOW);  
    // Activate internal pull-up
    digitalWrite(BUTTON_PIN,HIGH);
  
    present(CHILD_ID, S_DOOR);  
}

void report_state(int state)
{
    Serial.print("reporting state: ");
    Serial.println(state); 
    send(msg.set(state));
    if (state > 0)
      digitalWrite(LED_PIN,HIGH);  
    else
      digitalWrite(LED_PIN,LOW);  
}

int prev_state = -1;
int prev_state_length = 0;

//  Check if digital input has changed and send in new value
void loop() 
{
  
  // Get the update value
  int value = digitalRead(BUTTON_PIN);

  if (value != 0 && state != 1) 
  { // vibration detected for the first time during the period
    state = 1; 
    Serial.println("Vibration detected: ");
  }

  if (millis() - period_start > SAMPLING_PERIOD) 
  {
    period_start = millis();
    //send heartbeat to controller
    Serial.println("send heartbeat to controller");
    sendHeartbeat();
    
    Serial.print("State: ");
    Serial.println(state);
    if (state != prev_state) 
    {
      if (state == 1 && prev_state_length > 4) 
      { 
        Serial.println("Machine cycle Started:");
        //a vibration period after long period of quiet means a cycle started
        report_state(state);
        prev_state_length = 1;
      
      } 
    } 
    else if (state == 0 && prev_state_length > 2) 
    { 

      // a long period of quiet means cycle ended
      if (prev_state_length == 3)
      {
        Serial.println("Machine cycle ended:");
        report_state(state); // report only once 
      }
    }
    
    if (state != prev_state)
      prev_state_length = 1;
    else
      prev_state_length++;
  
    prev_state = state;
    state = 0;
  }

  delay(100);
} 
