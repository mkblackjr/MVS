/*
 * Sketch to control the motor for the Microscopic Video Surveillance System 
 * via Python User Interface
 *
 * Goal: Receive command(s) (steps to turn, speed to turn, multiple motor control) 
 * from Python program over serial connection and execute those commands in the
 * Arduino environment
 */

/*******************************************************************************
 ********************** Global Variable Declarations ***************************
 ******************************************************************************/

int steps; // Number of steps to turn the motor
int n = 0; // Assists in the looping of the turning sequences
String python_msg = "placeholder\n";
int wait_time = 1;
int R_steps; int Theta_steps; int Z_steps;
int R_start = 0; int Theta_start = 999; int Z_start = 999;
int wait_start = 999;

const int R_motor_pins[4] = {2, 3, 4, 5};
const int Theta_motor_pins[4] = {6, 7, 8, 9};
const int Z_motor_pins[4] = {10, 11, 12, 13};
int max_motor_pins[4]; int mid_motor_pins[4]; int min_motor_pins[4];

const int R_contact_pin = A0;
const int Theta_contact_pin = A1;
const int Z_contact_pin = A2;


// Turning sequence definitions for motor control
const int clockwise[8][4] = { // Clockwise turning sequence
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}, 
                      };
                      
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

/*******************************************************************************
 ***************************** Helper Functions ********************************
 ******************************************************************************/
 
// Function to initialize the input/output pins associated w/ motor control
void set_pin_mode(int motor_pins[], char mode){
    bool M;
    switch (mode) {
      case 'O':
         M = OUTPUT; break;
      case 'I':
         M = INPUT; break;
      case 'IP':
         M = INPUT_PULLUP; break;
    }
    pinMode(motor_pins[0], M);
    pinMode(motor_pins[1], M);
    pinMode(motor_pins[2], M);
    pinMode(motor_pins[3], M); 
}

// Turns off all motor pins
void turn_motors_off(int motor_pins[]){
    digitalWrite(motor_pins[0],0);
    digitalWrite(motor_pins[1],0);
    digitalWrite(motor_pins[2],0);
    digitalWrite(motor_pins[3],0);
}

// Receives motor pins and steps and turns motor accordingly 
void turn_motor(int motor_pins[], int n){
    if(n>0){
      digitalWrite(motor_pins[0],clockwise[n-1][0]);
      digitalWrite(motor_pins[1],clockwise[n-1][1]);
      digitalWrite(motor_pins[2],clockwise[n-1][2]);
      digitalWrite(motor_pins[3],clockwise[n-1][3]);
      delay(wait_time);
    }else {
      digitalWrite(motor_pins[0],counterclockwise[abs(n)-1][0]);
      digitalWrite(motor_pins[1],counterclockwise[abs(n)-1][1]);
      digitalWrite(motor_pins[2],counterclockwise[abs(n)-1][2]);
      digitalWrite(motor_pins[3],counterclockwise[abs(n)-1][3]);
      delay(wait_time);
    }        
}

// Receives step commands and computes max, median, and min in order
// to ensure that all commands are synchronized on a 0 to 1 map
void move_motors(int R, int Theta, int Z){
    // Determine absolute max, absolute median, absolute min
    int max_steps = signed_max(signed_max(R,Z), Theta);
    int min_steps = signed_min(signed_min(R,Z), Theta);
    int mid_steps = find_mid_steps(R, Theta, Z, max_steps, min_steps);

    // Loop over maximum number of steps -> median and min steps do
    // not execute on every loop, but are spaced out and finish in sync
    int num_loops = abs(max_steps);
    float mid_count = abs(round(max_steps/mid_steps)); 
    float min_count = abs(round(max_steps/min_steps));
    int max_turns = 0; int mid_turns = 0; int min_turns = 0;

    for(int i=0;i<num_loops;i++){

      // Execute movement for motor with farthest distance to travel (max)
      max_turns = signed_increment(max_turns, max_steps);
      turn_motor(max_motor_pins,max_turns);
      max_steps = signed_decrement(max_steps, max_steps);
      mid_count--; min_count--; 

      // Execute movement for motor with median distance to travel (mid)
      // Only execute on loop multiple of ratio of max to median
      if((mid_count == 0) & (mid_steps != 0)){
        mid_turns = signed_increment(mid_turns, mid_steps);
        turn_motor(mid_motor_pins,mid_turns);
        mid_steps = signed_decrement(mid_steps, mid_steps);
        if(mid_steps != 0){
          mid_count = round(abs(max_steps/mid_steps));
        }
      }

      // Execute movement for motor with least distance to travel (min)
      // Only execute on loop multiple of ratio of max to min
      if((min_count == 0) & (min_steps != 0)){
        min_turns = signed_increment(min_turns, min_steps);
        turn_motor(min_motor_pins,min_turns);
        min_steps = signed_decrement(min_steps, min_steps);
        if(min_steps != 0){
        min_count = round(abs(max_steps/min_steps));
        }
      }

      // Adjust for allowable turn sequence (0-7)
      max_turns = turn_limit(max_turns);
      mid_turns = turn_limit(mid_turns);
      min_turns = turn_limit(min_turns);
    }
}

// Increases absval of input
int signed_increment(int steps, int step_sign){
  steps += ((step_sign>0) - (step_sign<0));
  return steps;
}

// Decreases absval of input
int signed_decrement(int steps, int step_sign){
  steps -= ((step_sign>0) - (step_sign<0));
  return steps;
}

// Enforces limit of counter/clockwise turning routine
int turn_limit(int turns){
  if(abs(turns)==8){
        turns = 0;
  }
  return turns;
}

// Returns the input with the higher absval
int signed_max(int x, int y){
  if(abs(x)>abs(y)){
    return x;
  }else {return y;}
}

// Returns the input with the lower absval
int signed_min(int x, int y){
  if(abs(x)>abs(y)){
    return y;
  }else {return x;}
}

// Identifies the median number of steps by eliminating candidacy of min/max
int find_mid_steps(int R, int Theta, int Z, int max_steps, int min_steps){
  // Determine corresponding motor pins
    int mid_steps;
    bool r = 1; bool t = 1; bool z = 1;
    
    if(R==max_steps){
      r = 0;
      for(int i=0;i<4;i++){
        max_motor_pins[i] = R_motor_pins[i];
      }
    }else if(Theta==max_steps){
      t = 0;
      for(int i=0;i<4;i++){
        max_motor_pins[i] = Theta_motor_pins[i];
      }
    }else {
      z = 0;
      for(int i=0;i<4;i++){
        max_motor_pins[i] = Z_motor_pins[i];
      }
    }

    if(R==min_steps & r){
      r = 0;
      for(int i=0;i<4;i++){
        min_motor_pins[i] = R_motor_pins[i];
      }
    }else if(Theta==min_steps & t){
      t = 0;
      for(int i=0;i<4;i++){
        min_motor_pins[i] = Theta_motor_pins[i];
      }
    }else {
      z = 0;
      for(int i=0;i<4;i++){
        min_motor_pins[i] = Z_motor_pins[i];
      }
    }

    if(r){
      mid_steps = R;
      for(int i=0;i<4;i++){
        mid_motor_pins[i] = R_motor_pins[i];
      }
    }else if(t){
      mid_steps = Theta;
      for(int i=0;i<4;i++){
        mid_motor_pins[i] = Theta_motor_pins[i];
      }
    }else {
      mid_steps = Z;
      for(int i=0;i<4;i++){
        mid_motor_pins[i] = Z_motor_pins[i];
      }
    }

    return mid_steps;
}

// Deciphers step command and picks out R steps
int find_R_steps(String step_command){

  String c; String R_steps_string = "";
  Theta_start = 999;
  for(int i=(R_start+1); i<Theta_start; i++){
    c = step_command[i];
    if(c != "T"){
      R_steps_string.concat(c);
    }
    else{
      Theta_start = i;
    }
  }

  return R_steps_string.toInt();

}

// Deciphers step command and picks out Theta steps
int find_Theta_steps(String step_command){
  String c; String Theta_steps_string = "";
  Z_start = 999;
  for(int i=(Theta_start+1); i<Z_start; i++){
    c = step_command[i];
    if(c != "Z"){
      Theta_steps_string.concat(c);
    }
    else{
      Z_start = i;
    }
  }

  return Theta_steps_string.toInt();

}

// Deciphers step command and picks out Z steps
int find_Z_steps(String step_command){
  String c; String Z_steps_string = "";
  wait_start = 999;
  for(int i=(Z_start+1); i<wait_start; i++){
    c = step_command[i];
    if(c != "W"){
       Z_steps_string.concat(c);
    }
  }

  return Z_steps_string.toInt();

}

// Deciphers step command and picks out the wait time for speed enforcement
int find_wait_time(String step_command){
  String c; String wait_time_string = "";
  for(int i=(wait_start+1); i<step_command.length(); i++){
    c = step_command[i];
    if(c != ""){
       wait_time_string.concat(c);
    }
  }

  return wait_time_string.toInt();

}

// Brings the motor positions to (0,0,0) by using contact sensors
void home_motors(){

  bool at_home = false; 
  bool R_home = at_home; 
  bool Theta_home = at_home; 
  bool Z_home = at_home;
  int R; int Theta; int Z;
  while(!at_home){
    R = 0; 
    Theta = 0; 
    Z = 0;

    // Check each of the three contact sensors
    if (analogRead(R_contact_pin) <= 10){
      R_home = true;
    }else {R = -10;}
    if (analogRead(Theta_contact_pin) <= 10){
      Theta_home = true;
    }else {Theta = -10;}
    if (analogRead(Z_contact_pin) <= 10){
      Z_home = true;
    }else {Z = 10;} // Z direction is opposite other two
    if(R_home & Theta_home & Z_home){
      at_home = true;
    }

    move_motors(R, Theta, Z);
  
  }
  Serial.print("Go\n");
  delay(wait_time);
  Serial.print("Home Location (0,0,0) reached.\n");
}


/*******************************************************************************
 ***************************** Main Arduino Code *******************************
 ******************************************************************************/
 
void setup() {
    // Opens serial connection
    Serial.begin(9600); // Serial Port at 9600 baud
    Serial.setTimeout(50); // Instead of the default 1000ms, in order
                            // to speed up the Serial.parseInt()

    // Initializes motor pins and contact sensor pins
    set_pin_mode(R_motor_pins, 'O'); // Turn on the pins for R motor control
    set_pin_mode(Theta_motor_pins, 'O'); // Turn on the pins for Theta motor control
    set_pin_mode(Z_motor_pins, 'O'); // Turn on the pins for Z motor control
    pinMode(R_contact_pin, "INPUT");
    pinMode(Theta_contact_pin, "INPUT");
    pinMode(Z_contact_pin, "INPUT");
    
}

void loop() {

    // Wait for python to send the go ahead
    while (Serial.available() <= 0){}
    python_msg = Serial.readString();
    Serial.flush();
    Serial.print(python_msg);

    // Bring the motors to (0, 0, 0) before sending the go ahead
    home_motors();

    // Python waits for this signal to send a new command
    Serial.print("Go\n");
    String step_command = "";

    while(step_command != "QUIT"){
          
      // Read given step commands
      while(step_command[0] != 'R'){
        step_command = Serial.readString();
        delay(wait_time);
      }

      R_steps = find_R_steps(step_command);
      Theta_steps = find_Theta_steps(step_command);
      Z_steps = find_Z_steps(step_command);
      wait_time = find_wait_time(step_command) + 1;
      Serial.flush();
    
      // Execute commands
      move_motors(R_steps, Theta_steps, Z_steps);

      // Wait for next step commands, including flag for R, Theta, Z motors
      step_command = "";

      // Python waits for this signal to send a new command
      Serial.print("Go\n");
      while(Serial.available() <= 0){}
      step_command = Serial.readString();
  
      // Wait for full transmission to come through
      delay(wait_time);
    }
    Serial.print("Home\n");
    // Home the motors upon ending the Python program
    home_motors();

    Serial.flush();
    turn_motors_off(R_motor_pins);
    turn_motors_off(Theta_motor_pins);
    turn_motors_off(Z_motor_pins);
}


