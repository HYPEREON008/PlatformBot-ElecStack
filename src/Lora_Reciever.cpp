// #define RX PC_5 
// #define TX PC_4

char value;
#define M1 PA_6
#define M0 PA_5

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(PB_2,OUTPUT);
  pinMode(PB_7,OUTPUT);
  pinMode(PE_0,OUTPUT);
  pinMode(M0,OUTPUT);
  pinMode(M1,OUTPUT);

  digitalWrite(PB_2,HIGH);
  digitalWrite(PB_7,HIGH);
  digitalWrite(PE_0,HIGH);

  Serial.println("Ready to Recieve");
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  digitalWrite(M0,LOW);
  digitalWrite(M1,LOW);
    
    while (Serial1.available())
    {
      value = Serial1.read();
      

      if(value == '1')
      {
        digitalWrite(PB_7,LOW);
        digitalWrite(PE_0,LOW);
        Serial.println("Relays turned off");
      }

 //     else if(value == '1' && bot_state == false)
 //     {
 //       digitalWrite(PB_2,HIGH);
 //       digitalWrite(PB_7,HIGH);
 //       digitalWrite(PE_0,HIGH);
 //       Serial.println("Relays turned on");
        
 //     }

    


}


}