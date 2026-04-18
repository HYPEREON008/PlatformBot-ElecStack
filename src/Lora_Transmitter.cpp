#define RX PD_655                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        semem
#define TX PD_7   
#define M1 PA_5
#define M0 PA_6
bool bot_state = true;
bool lastButtonState = LOW;  
bool currentButtonState = LOW;

unsigned long lastTime = 0;
const unsigned long Delay = 20; 

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);          
  Serial.println("TIVA Initialized");
  pinMode(PF_2, INPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
}

void loop() {
  int reading = digitalRead(PF_2);

  if (reading != lastButtonState) {
    lastTime = millis(); 
  }

  if ((millis() - lastTime) > Delay) {
    
    if (reading == HIGH && currentButtonState == LOW) {
      if (bot_state) {
        Serial2.print('1');
        Serial.println("Bot Stop data sent");
        bot_state = false;
      } else {
        Serial2.print('1');
        Serial.println("Bot On data sent");
        bot_state = true;
      }
    }
    currentButtonState = reading;
  }

  lastButtonState = reading;
}