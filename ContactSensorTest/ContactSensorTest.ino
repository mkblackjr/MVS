int R_pin = A0;
int Theta_pin = A1;
int Z_pin = A2;
int R_value = 0;
int Theta_value = 0;
int Z_value = 0;
const int counterclockwise[8][4] = { // Counterclockwise turning sequence             
  {1, 0, 0, 1},
  {0, 0, 0, 1},
  {0, 0, 1, 1},
  {0, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 0},
  {1, 1, 0, 0},
  {1, 0, 0, 0}, 
                      };


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Serial Port at 9600 baud
  Serial.setTimeout(50);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  
//  pinMode(5, OUTPUT);
//  pinMode(6, OUTPUT);
//  pinMode(7, OUTPUT);
//  pinMode(8, OUTPUT);
//  pinMode(10, OUTPUT);
//  pinMode(11, OUTPUT);
//  pinMode(12, OUTPUT);
//  pinMode(13, OUTPUT);
}

void loop() {
//  // put your main code here, to run repeatedly:
bool go = true;

while(go){
  for(int i = 0; i<8; i++){
    digitalWrite(2,counterclockwise[i][0]);
    digitalWrite(3,counterclockwise[i][1]);
    digitalWrite(4,counterclockwise[i][2]);
    digitalWrite(5,counterclockwise[i][3]);
    if(Serial.available()>0){
      go = false;
    }
    delay(3);
  }
}
delay(1000);
//  digitalWrite(4, 1);
//  digitalWrite(5, 1);
//  digitalWrite(6, 1);
//  digitalWrite(7, 1);
//  digitalWrite(8, 1);
//  digitalWrite(9, 1);
//  digitalWrite(10, 1);
//  digitalWrite(11, 1);
//  digitalWrite(12, 1);
//  digitalWrite(13, 1);
  delay(1000);
//  digitalWrite(2, 0);
//  digitalWrite(3, 0);
  digitalWrite(4, 0);
  digitalWrite(5, 0);
//  digitalWrite(6, 0);
//  digitalWrite(7, 0);
//  digitalWrite(8, 0);
//  digitalWrite(9, 0);
//  digitalWrite(10, 0);
//  digitalWrite(11, 0);
//  digitalWrite(12, 0);
//  digitalWrite(13, 0);

  Serial.print(analogRead(R_pin));
  Serial.print("R\n");
  Serial.print(analogRead(Theta_pin));
  Serial.print("T\n");
  Serial.print(analogRead(Z_pin));
  Serial.print("Z\n");
}
