/**
 * Use references from SparkFun Inventor's Kit Project:
 * 
*/

// Stop counting after this value 
const unsigned int MAX_T = 20000;

// Pin definitions
const int P_JNCT_PIN_R = 2;// P junction of sensing LED
const int N_JNCT_PIN_R = 3;// N junction of sensing LED

const int P_JNCT_PIN_G = 4;// P junction of sensing LED
const int N_JNCT_PIN_G = 5;// N junction of sensing LED

uint16_t send_buffer[2];
uint8_t delimiter [] = {0x80};

#define RED 0
#define GREEN 1

void setup() {

  // Start serial communications
  Serial.begin(115200);

  // Set P junction pin to output low (GND)
  pinMode(P_JNCT_PIN_R, OUTPUT);
  digitalWrite(P_JNCT_PIN_R, LOW);

  pinMode(P_JNCT_PIN_G, OUTPUT);
  digitalWrite(P_JNCT_PIN_G, LOW);

}

void loop() {

  // Read the amount of light falling on the LED
  uint16_t redt =  readLED(RED);
  uint16_t greent =  readLED(GREEN);

  send_buffer[0] = escapeTerminator(redt);
  send_buffer[1] = escapeTerminator(greent);
  
  // Print out the raw discharge time
  senddata();
}

int readLED(int color) {
  int  sen_time = 0;
  unsigned int t;
  
  if(color == RED){
    // Apply reverse voltage to charge the sensing LED's capacitance
    pinMode(N_JNCT_PIN_R, OUTPUT);
    digitalWrite(N_JNCT_PIN_R, HIGH);
  
    // Isolate N junction and turn off pull-up resistor
    pinMode(N_JNCT_PIN_R, INPUT);
    digitalWrite(N_JNCT_PIN_R, LOW);
  
    // Count how long it takes for the LED to discharge
    for ( t = 0; t < MAX_T; t++ ) {
      if ( digitalRead(N_JNCT_PIN_R) == 0 ) {
        break;
      }
    }
  }else if(color == GREEN){

    // Apply reverse voltage to charge the sensing LED's capacitance
    pinMode(N_JNCT_PIN_G, OUTPUT);
    digitalWrite(N_JNCT_PIN_G, HIGH);
  
    // Isolate N junction and turn off pull-up resistor
    pinMode(N_JNCT_PIN_G, INPUT);
    digitalWrite(N_JNCT_PIN_G, LOW);
  
    // Count how long it takes for the LED to discharge
    for ( t = 0; t < MAX_T; t++ ) {
      if ( digitalRead(N_JNCT_PIN_G) == 0 ) {
        break;
      }
    }
    
  }

  sen_time = t;

  return sen_time;
}

void senddata(){
  Serial.write((uint8_t *) send_buffer, sizeof(send_buffer));
  Serial.write((uint8_t *)delimiter, sizeof(delimiter));
}

static inline uint16_t escapeTerminator (uint16_t v){
  if((v & 0xff) == 0x80){
    v++;
  }

  if((v >> 8) == 0x80){
    v += 0x100 - (v & 0xff); 
  }
  return v;
}
