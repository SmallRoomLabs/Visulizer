void Banner(void) {
  Serial.println("");
  Serial.println(" _  _ _ ____ _  _ _    _ ___  ____ ____");
  Serial.println(" |  | | [__  |  | |    |   /  |___ |__/");
  Serial.println("  \\/  | ___] |__| |___ |  /__ |___ |  \\");
  Serial.println("Setup version 1.0");
  Serial.println("(Don't forget to set Line Ending Mode to Carriage Return)");
  Serial.println();
}                                                                             


int32_t Menu(uint8_t *arg) {
  char buf[10];
  uint8_t len;
  int32_t v;
  int32_t m;
  
  Serial.println();
  Serial.println("0 CR   - Exit setup mode ");
  Serial.println("1 <value> CR - Set standard low level 0-255, normal value is 10");
  Serial.println("2 <value> CR - Set standard high level 0-255, normal value is 245");
  Serial.println("3 <value> CR - Set alternate low level 0-255");
  Serial.println("4 <value> CR - Set alternate high level 0-255");
  Serial.println("5 0 CR - Activate attract mode at start ");
  Serial.println("5 1 CR - Deactivate attract mode at start ");
  Serial.println("6 CR - Arduino all input ");
  Serial.println("7 CR - Sample blinking ");
  Serial.print("cmd>");

  len=0;
  while (1) {
    if (Serial.available() > 0) {
      buf[len]=Serial.read();
      if (buf[len]==13) {
        break;
      }
    len++;
    }
  }
  buf[len]=0;
  m=atol(buf);
  v=atol(&buf[2]);
  *arg=(uint8_t)v;
  return m;
}


void AllOutput(void) {
  int i;
  for (i=0; i<20; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
}

void AllInput(void) {
  int i;

  for (i=0; i<20; i++) {
    pinMode(i, INPUT);
    digitalWrite(i, LOW);  // No pullup
  }
}


void setup() {                
  Serial.begin(9600);
  Banner();
}

#define SENDCLK        2
#define SENDFIRSTDATA  6


void SendByte(uint8_t b) {
  uint8_t i;
  
  digitalWrite(SENDCLK, LOW);
  for (i=0; i<8; i++) {
      if (b&(1<<i)) digitalWrite(SENDFIRSTDATA+i, HIGH); 
      else digitalWrite(SENDFIRSTDATA+i, LOW);
    }    
  delay(100);
  digitalWrite(SENDCLK, HIGH);
  delay(100);
  digitalWrite(SENDCLK, LOW);
}


void SendTrigger(void) {
    SendByte('V');
    SendByte('i');
    SendByte('s');
    SendByte('u');
    SendByte('l');
    SendByte('i');
    SendByte('z');
    SendByte('e');
    SendByte('r');
}


void loop() {
  int i,j,m;
  uint8_t b;
  uint8_t v;
  
  m=Menu(&v);
  Serial.println();
  AllOutput();
  if (m==0) {
    Serial.println("Exit Setup");
    SendTrigger();
    SendByte(1);
    SendByte(0);
  }
  if (m==5) {
    Serial.println("Updating attract mode");
    SendTrigger();
    SendByte(2);
    if (v==1) SendByte(1); else SendByte(0);
  }
  if (m==1) {
    Serial.print("Setting Standard low level to ");
    Serial.println(v);
    SendTrigger();
    SendByte(3);
    SendByte(v);
  }
  if (m==2) {
    Serial.print("Setting Standard high level to ");
    Serial.println(v);
    SendTrigger();
    SendByte(4);
    SendByte(v);
  }
  if (m==3) {
    Serial.print("Setting Alternate low level to ");
    Serial.println(v);
    SendTrigger();
    SendByte(5);
    SendByte(v);
  }
  if (m==4) {
    Serial.print("Setting Alternate high level to ");
    Serial.println(v);
    SendTrigger();
    SendByte(6);
    SendByte(v);
  }

  if (m==6) {
    Serial.println("All Arduino pins as input - no pullup ");
    AllInput();
    for (;;);
  }  

  if (m==7) {
  
    AllOutput();
    for (i=0; i<20; i++) {
      digitalWrite(i, HIGH);
      delay(250);
      digitalWrite(i, LOW);
    }

    for (i=0; i<20; i++) {
      digitalWrite(i, HIGH);
      delay(250);
    }

    for (i=0; i<20; i++) {
      digitalWrite(i, LOW);
      delay(250);
    }

    for (j=0; j<4; j++) {
      AllInput();
      delay(2000);
      AllOutput();    
      for (i=0; i<20; i++) digitalWrite(i, HIGH);
      delay(250);
      for (i=0; i<20; i++) digitalWrite(i, LOW);
      delay(250);
    }
  }

}


