import time

def turn_motor(motor_pins, n):
  if(n>0):
    # print(motor_pins[0],counterclockwise[n-1][0]);
    # print(motor_pins[1],counterclockwise[n-1][1]);
    # print(motor_pins[2],counterclockwise[n-1][2]);
    # print(motor_pins[3],counterclockwise[n-1][3]);
    # print("\n")
    time.sleep(.001);
  else:
    # print(motor_pins[0],clockwise[abs(n)-1][0]);
    # print(motor_pins[1],clockwise[abs(n)-1][1]);
    # print(motor_pins[2],clockwise[abs(n)-1][2]);
    # print(motor_pins[3],clockwise[abs(n)-1][3]);
    # print("\n")
    time.sleep(.001);


def move_motors(R, Theta, Z):
  max_steps = signed_max(signed_max(R,Z), Theta);
  min_steps = signed_min(signed_min(R,Z), Theta);
  mid_steps = find_mid_steps(R, Theta, Z, max_steps, min_steps);
  num_loops = abs(max_steps);
  mid_count = abs(round(max_steps/mid_steps)); 
  min_count = abs(round(max_steps/min_steps));
  max_turns = 0; mid_turns = 0; min_turns = 0;

  while (num_loops!=0): 

    # Execute movement for motor with farthest distance to travel (max)
    num_loops -= 1
    max_turns = signed_increment(max_turns, max_steps);
    turn_motor(max_motor_pins,max_turns);
    max_steps = signed_decrement(max_steps, max_steps);
    mid_count -= 1; min_count -= 1; 


    # Execute movement for motor with median distance to travel (mid)
    if(mid_count == 0 and mid_steps != 0):
      mid_turns = signed_increment(mid_turns, mid_steps);
      turn_motor(mid_motor_pins,mid_turns);
      mid_steps = signed_decrement(mid_steps, mid_steps);
      if(mid_steps != 0):
        mid_count = round(abs(max_steps/mid_steps));


    # Execute movement for motor with least distance to travel (min)
    if(min_count == 0 and min_steps != 0):
      min_turns = signed_increment(min_turns, min_steps);
      turn_motor(min_motor_pins,min_turns);
      min_steps = signed_decrement(min_steps, min_steps);
      if(min_steps != 0):
        min_count = round(abs(max_steps/min_steps));


    # Adjust for allowable turn sequence (0-7)
    max_turns = turn_limit(max_turns);
    mid_turns = turn_limit(mid_turns);
    min_turns = turn_limit(min_turns);

    print("max_steps = " + str(max_steps))
    print("mid_steps = " + str(mid_steps))
    print("min_steps = " + str(min_steps))


def signed_increment(steps, step_sign):
  steps += ((step_sign>0) - (step_sign<0));
  return steps;

def signed_decrement(steps, step_sign):
  steps -= ((step_sign>0) - (step_sign<0));
  return steps;

def turn_limit(turns):
  if(abs(turns)==8):
    turns = 0;
  return turns;

def signed_max(x, y):
  if(abs(x)>abs(y)):
    return x;
  else: return y;

def signed_min(x, y):
  if(abs(x)<abs(y)):
    return x;
  else: return y;

def find_mid_steps(R, Theta, Z, max_steps, min_steps):
  global max_motor_pins
  global min_motor_pins
  global mid_motor_pins

  # Determine corresponding motor pins
  i = 0;
  if(R==max_steps):
    while(i<4):
      max_motor_pins[i] = R_motor_pins[i];
      i += 1;
  elif(Theta==max_steps):
    while(i<4):
      max_motor_pins[i] = Theta_motor_pins[i];
      i += 1;
  else:
    while(i<4):
      max_motor_pins[i] = Z_motor_pins[i];
      i += 1;

  i = 0;
  if(R==min_steps):
    while(i<4):
      min_motor_pins[i] = R_motor_pins[i];
      i += 1;
  elif(Theta==min_steps):
    while(i<4):
      min_motor_pins[i] = Theta_motor_pins[i];
      i += 1;
  else:
    while(i<4):
      min_motor_pins[i] = Z_motor_pins[i];
      i += 1;

  i = 0;
  if(R!=max_steps and R!=min_steps):
    mid_steps = R;
    while(i<4):
      mid_motor_pins[i] = R_motor_pins[i];
      i += 1;
  elif(Theta!=max_steps and Theta!=min_steps):
    mid_steps = Theta;
    while(i<4):
      mid_motor_pins[i] = Theta_motor_pins[i];
      i += 1;
  else:
    mid_steps = Z;
    while(i<4):
      mid_motor_pins[i] = Z_motor_pins[i];
      i += 1;

  return mid_steps;

###############################################################################
step_default = 510; # Default movement is one turn of motor

R_motor_pins = [2, 3, 4, 5];
Theta_motor_pins = [6, 7, 8, 9];
Z_motor_pins = [10, 11, 12, 13];

max_motor_pins = [0, 0, 0, 0]
mid_motor_pins = [0, 0, 0, 0]
min_motor_pins = [0, 0, 0, 0]

# Turning sequence definitions for motor control
clockwise = [ # Clockwise turning sequence
  [1, 0, 0, 0],
  [1, 1, 0, 0],
  [0, 1, 0, 0],
  [0, 1, 1, 0],
  [0, 0, 1, 0],
  [0, 0, 1, 1],
  [0, 0, 0, 1],
  [1, 0, 0, 1], 
                      ];
                      
counterclockwise = [ # Counterclockwise turning sequence             
  [1, 0, 0, 1],
  [0, 0, 0, 1],
  [0, 0, 1, 1],
  [0, 0, 1, 0],
  [0, 1, 1, 0],
  [0, 1, 0, 0],
  [1, 1, 0, 0],
  [1, 0, 0, 0], 
                      ]; 

move_motors(-100,20,-30)
