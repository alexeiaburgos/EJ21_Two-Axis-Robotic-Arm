//This program was made by Gabriel Rodgers for the 2-axis Scara Robotic Arm Project in 
//Junior Design 2 (ECE 342)

//header files and libraries
#include <stdio.h>
#include <string.h>

//lesson: 
//DON'T MALLOC SPACE IN A FUNCTION THEN RETURN THAT VALUE HELD IN THAT SPACE TO ANOTHER PREVIOUSLY MALLOCED SPOT
//DON'T MALLOC GLOBAL VARS

//define pins for Output to motor driver:
// Note that I don't need to program the VDD pin or the GND pin
// VDD: Pin 5
// GND: Pin 6 or Pin 7
//Stepper motor pin for motor 1 will be Pin 2
//Step motor pin for m2 will be Pin 4
uint8_t m1Step = 2; //D1
uint8_t m2Step = 4; //D3

//Direction motor pin for motor 1 will be Pin 3
//Dir motor pin for m2 will be Pin 5
uint8_t m1Dir = 3; //D2
uint8_t m2Dir = 5; //D4

//define an array of integers that will hold the current x,y coordinates 
//define it as volatile because this global is accessed at different areas of the code
//[current x-coord, current y-coord]
volatile double currCoords[2];

//define an int to state the coordinate mode (relative or absolute)
uint8_t coordMode = 1; //initialize to absolute mode
//1 = absolute mode
//0 = relative mode

//define an int to state the unit type (inches or millimeters)
uint8_t unitMode = 0; //initialize to millimeters
//1 = inches
//0 = millimeters

//define an int to state the drawing mode (1: free draw mode. 0: g-code mode.)
uint8_t moveMode = 0; //initialize to g-code mode

//notes: 
// must stop the python script before downloading this program to the arduino
// when the python script is running, cannot open serial monitor in this IDE because the python script is
// connected to the arduino

//create a function that calculates the resulting angles from the given coordinates
double inverseKinematics(double* coordinates, uint8_t value) {
  //this function will return the angle corresponding to the value and coordinates given (basically this function needs to be called
  //2 times with the same coordinates to return the two arm angles)
  double angles[2];

  //clear out the angles array
  memset((void*) angles, '\0', 2*sizeof(double)); 

  //arm sizes, where A is the inner arm length and B is the outer arm length
  double A = 158; 
  double B = 148;

  //first calculate the q2 angle (from q1 axis to the second arm)
  angles[1] = acos(((pow(coordinates[0], 2) + pow(coordinates[1], 2) - pow(A, 2) - pow(B, 2))/(2*A*B)));
/*
  //for whatever reason, this kind of destroys the system... makes the line not straight
  if ((coordinates[0] == 0) && (coordinates[1] == 0)){
    //then calculate q1 angle (from x axis to the first arm)
    angles[0] = 0;
  }

  else {
    //then calculate q1 angle (from x axis to the first arm)
    angles[0] = atan(coordinates[1]/coordinates[0]) - atan((B*sin(angles[1]))/(A+(B*cos(angles[1]))));
  }
*/

  angles[0] = atan(coordinates[1]/coordinates[0]) - atan((B*sin(angles[1]))/(A+(B*cos(angles[1]))));

  return angles[value]; 
}

//create a function that calculates the resulting coordinates from the given angles
double forwardKinematics(double* angles, uint8_t value) {
  //this function will return the coordinate in the coordinates array indexed by the parameter value by calculating
  // the coordinates resulting from the given arm angles

  //first initialize an array of coordinates:
  double coordinates[2]; 

  //clear out the coordinates array
  memset((void*) coordinates, '\0', 2*sizeof(double)); 

  //arm sizes, where A is the inner arm length and B is the outer arm length
  double A = 158; 
  double B = 148;

  //then calculate the x value of the coordinates
  coordinates[0] = (A*cos(angles[0])) + (B*cos(angles[0] + angles[1]));

  //then calculate the y value of the coordinates
  coordinates[1] = (A*sin(angles[0])) + (B*sin(angles[0] + angles[1]));

  return coordinates[value];
}

void move(uint8_t* movementArray) {
  //this function will output a set of signals based on the movementArray provided

  //first do the inner motor movement (m1Step and m1Dir)
  //check for if a step is needed or not for m1
  if (movementArray[0] == 1) { //if m1 step is 1 then a movement is indicated

    if (movementArray[1] == 1) { //if m1Dir is 1, then do a clockwise movement

        //for clockwise movement:
        digitalWrite(m1Dir, HIGH); //set m1Dir to low for the first 6 us

        delayMicroseconds(300); //delay by 3 us

        digitalWrite(m1Step, HIGH); //for 15 us

        delayMicroseconds(300); // for 3 us

        digitalWrite(m1Dir, LOW); //this concludes the dir output

        delayMicroseconds(1200); //now it has been 15 us of the step out

        digitalWrite(m1Step, LOW); 

        delayMicroseconds(1500); //this concludes the step output
    }

    else { //if m1Dir is 0, then do a counter clockwise movement

      digitalWrite(m1Dir, LOW); //set m1Dir to low for the first 6 us

      delayMicroseconds(300);  //delay by 3 us

      digitalWrite(m1Step, HIGH); //for 15 us

      delayMicroseconds(300); // for 3 us

      digitalWrite(m1Dir, HIGH); //this concludes the dir output

      delayMicroseconds(1200); //now it has been 15 us of the step out

      digitalWrite(m1Step, LOW); 

      delayMicroseconds(1500); //this concludes the step output

    }
  }

  if (movementArray[2] == 1) { //if m2 step is 1 then a movement is indicated

    if (movementArray[3] == 1) { //if m2Dir is 1, then do a clockwise movement

        //for clockwise movement:
        digitalWrite(m2Dir, HIGH); //set m1Dir to low for the first 6 us

        delayMicroseconds(300); //delay by 3 us

        digitalWrite(m2Step, HIGH); //for 15 us

        delayMicroseconds(300); // for 3 us

        digitalWrite(m2Dir, LOW); //this concludes the dir output

        delayMicroseconds(1200); //now it has been 15 us of the step out

        digitalWrite(m2Step, LOW); 

        delayMicroseconds(1500); //this concludes the step output
    }

    else { //if m2Dir is 0, then do a counter clockwise movement

      digitalWrite(m2Dir, LOW); //set m1Dir to low for the first 6 us

      delayMicroseconds(300);  //delay by 3 us

      digitalWrite(m2Step, HIGH); //for 15 us

      delayMicroseconds(300); // for 3 us

      digitalWrite(m2Dir, HIGH); //this concludes the dir output

      delayMicroseconds(1200); //now it has been 15 us of the step out

      digitalWrite(m2Step, LOW); 

      delayMicroseconds(1500); //this concludes the step output
    }
  }
  //note: the last delay might not be needed
  //delay(10);

}

double calculateDistance(double* src, double* dest){
  //this function will calculate the distance between the two points given
  double d = pow((pow((dest[0] - src[0]), 2) + pow((dest[1] - src[1]), 2)), 0.5);

  return d;
}

void moveThis() {
    //This function basically tests if the signals will move the motor
  
      digitalWrite(m1Dir, LOW); //set m1Dir to low for the first 6 us

      int i = 0;

      for (i; i < 200; i++) {

        delayMicroseconds(300);  //delay by 3 us

        digitalWrite(m1Step, HIGH); //for 15 us

        delayMicroseconds(300); // for 3 us

        digitalWrite(m1Dir, HIGH); //this concludes the dir output

        delayMicroseconds(1200); //now it has been 15 us of the step out

        digitalWrite(m1Step, LOW); 

        delayMicroseconds(1500); //this concludes the step output

      }
}


//create function to do a movement in a line and return the coordinates after the movement
void kinematics(double* source, double* destination) {
  //this function will calculate the movement required for a straight line from source to destination coordinates
  //and also send the signals required for that straight line to the move function. 
  
  //the parameters for this function are an array of integers holding the current coordinates
  // and an array of integers holding the destination coordinates

  //initialize variables
  uint32_t idx; //iteration integer

  //create a double to hold the angle in radians of a single step (assuming 1/8 step: 0.001309)
  double angle = 0.001309*8*2; 

  //create a variable that holds the distance from the source to the destination coordinates
  double realDistance;

  //create an array of angles to hold the angles
  double sourceAngles[2];

  //create an array of angles to hold manipulated angle measurements
  double checkAngles[2];

  double bestDistance; //this value will hold the distance that is closest to the destination coordinates
  double checkDistance; //this value will hold the distance from the new movement coordinates to the end coordinates

  //initalize some arrays to hold coordinates
  double bestMoveCoords[2]; 
  double checkMoveCoords[2]; 

  //if you remove this bad things will happen
  memset(bestMoveCoords, '/0', 2*sizeof(double)); //eventually want to change /0 to \0 but cant rn

  uint8_t bestMoveOutputs[4]; //this will hold the movement outputs for the closest to realSlope movement
  uint8_t checkMoveOutputs[4]; //this will hold the current movement outputs for a set movement

  //enter a while loop that will loop until the calculated source coordinates are within a set range of the destination coordinates
  while (1) {

    //set some vars
    idx = 0; //set the iteration integer to 0

    //calulate the distance from source to destination coordinates
    realDistance = calculateDistance(source, destination); 

    //calculate the sourceAngles (angles of the arm at the source coordinates)
    sourceAngles[0] = inverseKinematics(source, 0); //populate the source angles with values
    sourceAngles[1] = inverseKinematics(source, 1);

    bestDistance = realDistance; //this value will hold the distance that is closest to the destination coordinates

    //do some memcpy so that we don't loop forever doing nothing
    memcpy(checkMoveCoords, source, 2*sizeof(double));

    //cw movement: subtract angle from angles
    //ccw movement: add angle to angles

    for (idx=0; idx < 8; idx++) { //loop 8 times for each possible movement
      
      //clear out the checkMoveCoords and checkMoveOutputs each iteration
      //check if this memset is even needed lol
      //memset(checkMoveOutputs, '/0', 4*sizeof(uint8_t)); //may not send outputs out or something!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! before i used '/0'

      //use switch case statement for speed 
      switch(idx) {

        //check the movement for m1 and m2 cw step
        case  0: 
          checkAngles[0] = sourceAngles[0] - angle; 
          checkAngles[1] = sourceAngles[1] - angle;

          //calculate the coordinates after the move
          checkMoveCoords[0] = forwardKinematics(checkAngles, 0); 
          checkMoveCoords[1] = forwardKinematics(checkAngles, 1); 

          //calculate the distance between the checkMoveCoords and the destination
          checkDistance = calculateDistance(checkMoveCoords, destination); 

          //set the checkMoveOutputs to the indicated signals
          checkMoveOutputs[0] = 1; //m1Step; 1 = 1 step, 0 = 0 steps
          checkMoveOutputs[1] = 1; //m1Dir; 1 = cw, 0 = ccw
          checkMoveOutputs[2] = 1; //m2Step; 1 = 1 step, 0 = 0 steps
          checkMoveOutputs[3] = 1; //m2Dir; 1 = cw, 0 = ccw
          break;

        case  1: //cw rotation for m1, no rotation for m2
          checkAngles[0] = sourceAngles[0] - angle; 
          checkAngles[1] = sourceAngles[1];

          //calculate the coordinates after the move
          checkMoveCoords[0] = forwardKinematics(checkAngles, 0); 
          checkMoveCoords[1] = forwardKinematics(checkAngles, 1); 

          //calculate the distance between the checkMoveCoords and the destination
          checkDistance = calculateDistance(checkMoveCoords, destination); 

          //set the checkMoveOutputs to the indicated signals
          checkMoveOutputs[0] = 1; 
          checkMoveOutputs[1] = 1;
          checkMoveOutputs[2] = 0;
          checkMoveOutputs[3] = 0; 
          break;

        case  2: //cw rotation for m1, ccw rotation for m2
          checkAngles[0] = sourceAngles[0] - angle; 
          checkAngles[1] = sourceAngles[1] + angle;

          //calculate the coordinates after the move
          checkMoveCoords[0] = forwardKinematics(checkAngles, 0); 
          checkMoveCoords[1] = forwardKinematics(checkAngles, 1); 

          //calculate the distance between the checkMoveCoords and the destination
          checkDistance = calculateDistance(checkMoveCoords, destination); 

          //set the checkMoveOutputs to the indicated signals
          checkMoveOutputs[0] = 1; 
          checkMoveOutputs[1] = 1;
          checkMoveOutputs[2] = 1;
          checkMoveOutputs[3] = 0; 
          break;
        
        case  3:  //ccw rotation for m1, cw movement for m2
          checkAngles[0] = sourceAngles[0] + angle; 
          checkAngles[1] = sourceAngles[1] - angle;

          //calculate the coordinates after the move
          checkMoveCoords[0] = forwardKinematics(checkAngles, 0); 
          checkMoveCoords[1] = forwardKinematics(checkAngles, 1); 

          //calculate the distance between the checkMoveCoords and the destination
          checkDistance = calculateDistance(checkMoveCoords, destination); 

          //set the checkMoveOutputs to the indicated signals
          checkMoveOutputs[0] = 1; 
          checkMoveOutputs[1] = 0;
          checkMoveOutputs[2] = 1;
          checkMoveOutputs[3] = 1; 
          break;

        case  4: //ccw rotation for m1, no rotation for m2
          checkAngles[0] = sourceAngles[0] + angle; 
          checkAngles[1] = sourceAngles[1];

          //calculate the coordinates after the move
          checkMoveCoords[0] = forwardKinematics(checkAngles, 0); 
          checkMoveCoords[1] = forwardKinematics(checkAngles, 1);  

          //calculate the distance between the checkMoveCoords and the destination
          checkDistance = calculateDistance(checkMoveCoords, destination); 

          //set the checkMoveOutputs to the indicated signals
          checkMoveOutputs[0] = 1; 
          checkMoveOutputs[1] = 0;
          checkMoveOutputs[2] = 0;
          checkMoveOutputs[3] = 0; 
          break;

        case  5: //ccw rotation for m1, ccw rotation for m2
          checkAngles[0] = sourceAngles[0] + angle; 
          checkAngles[1] = sourceAngles[1] + angle;

          //calculate the coordinates after the move
          checkMoveCoords[0] = forwardKinematics(checkAngles, 0); 
          checkMoveCoords[1] = forwardKinematics(checkAngles, 1); 

          //calculate the distance between the checkMoveCoords and the destination
          checkDistance = calculateDistance(checkMoveCoords, destination); 

          //set the checkMoveOutputs to the indicated signals
          checkMoveOutputs[0] = 1; 
          checkMoveOutputs[1] = 0;
          checkMoveOutputs[2] = 1;
          checkMoveOutputs[3] = 0; 
          break;

        case  6:  //no rotation for m1, cw rotation for m2
          checkAngles[0] = sourceAngles[0]; 
          checkAngles[1] = sourceAngles[1] - angle;

          //calculate the coordinates after the move
          checkMoveCoords[0] = forwardKinematics(checkAngles, 0); 
          checkMoveCoords[1] = forwardKinematics(checkAngles, 1); 

          //calculate the distance between the checkMoveCoords and the destination
          checkDistance = calculateDistance(checkMoveCoords, destination); 

          //set the checkMoveOutputs to the indicated signals
          checkMoveOutputs[0] = 0; 
          checkMoveOutputs[1] = 0;
          checkMoveOutputs[2] = 1;
          checkMoveOutputs[3] = 1; 
          break;

        case  7: //no rotation for m1, ccw rotation for m2
          checkAngles[0] = sourceAngles[0]; 
          checkAngles[1] = sourceAngles[1] + angle;

          //calculate the coordinates after the move
          checkMoveCoords[0] = forwardKinematics(checkAngles, 0); 
          checkMoveCoords[1] = forwardKinematics(checkAngles, 1); 

          //calculate the distance between the checkMoveCoords and the destination
          checkDistance = calculateDistance(checkMoveCoords, destination); 

          //set the checkMoveOutputs to the indicated signals
          checkMoveOutputs[0] = 0; 
          checkMoveOutputs[1] = 0;
          checkMoveOutputs[2] = 1;
          checkMoveOutputs[3] = 0; 
          break; 
      }

      //after a calculation, figure out if the movement is optimal or not
      if (checkDistance <= bestDistance) { //if the distance between the new movement coords and the destination is less than the previous best movement, then update the best movement
        bestDistance = checkDistance; //update best distance 
        memcpy(bestMoveOutputs, checkMoveOutputs, 2*sizeof(double)); //update bestMoveOutputs
        memcpy(bestMoveCoords, checkMoveCoords, 2*sizeof(double)); //update bestMoveCoords
      }
    }

    //out of the for loop

    //call the move function to send the outputs to the motor driver
    //do a delay after each call if lower speed required... figure out an equation to do that or something
    move(bestMoveOutputs); 
    move(bestMoveOutputs); //this is for double speed or shalf as accurate

    memcpy(source, bestMoveCoords, 2*sizeof(double)); 
    
    //for debugging purposes:

    Serial.print(source[0]); 
    Serial.print(", "); 
    Serial.print(source[1]);
    Serial.print("\n"); 
    Serial.flush(); 


    //if the distance from source to destination coordinates reaches a set limit, end the movement
    if (bestDistance < 0.2*8*2) {
      break;
    }
  }

  //out of the while loop

  //update the currCoords global variable
  currCoords[0] = source[0];
  currCoords[1] = source[1];
}

void setup() {
  // put your setup code here, to run once:
  // we  want to set up the serial port:
  Serial.begin(9600); //baud rate

  //set up a pin (LED pin) for output based on the input received through the serial port:
  pinMode(LED_BUILTIN, OUTPUT);

  //set up the four movement pins for output
  pinMode(m1Step, OUTPUT);
  pinMode(m2Step, OUTPUT);
  pinMode(m1Dir, OUTPUT); 
  pinMode(m2Dir, OUTPUT); 

  //initalize the initial coordinates
  currCoords[0] = 1;
  currCoords[1] = 1;
}

void loop() {

  //create some coordinates to tamper with
  double src[2];
  double dst[2];

  //define a variable that will hold the current G-code command from the py script
  char currCommand[30]; 
  char** commandArray = malloc(4*sizeof(currCommand));  //create an array of char* to store each part of the command
  char commandCopy[30]; //create a char[] to store a copy of currCommand

  double destAngles[2]; //create an array of doubles that will hold the angles of the arm 
  
  //use if statement to do something to the arduino once an input has been received
  if (Serial.available() >= 0) {
  
    //read the incoming data onto currCommand
    //read the data from the Serial pipeline: stop the string after the newline
    String strCommand = Serial.readStringUntil('\n'); //.c_str() converts the string to a char*

    //clear the currCommand and commandCopy
    memset((void*) currCommand, '\0', 30);
    memset((void*) commandCopy, '\0', 30);
    memset((void*) commandArray, '\0', 4*sizeof(currCommand));

    //initialize some vars: i is an iteration variable and strLen holds the length of the command string
    uint8_t i = 0;
    uint8_t strLen = strCommand.length();

    //convert the string to a char* using a for loop
    for (i = 0; i<strLen; i++){ 
      currCommand[i] = strCommand[i];
    }

    //reset the iteration variable 
    i=0;
  
    //start tokenization
    strcpy(commandCopy, currCommand);

    char* saveptr; // create variable for string copying

    //create the first token, delimited by spaces, \n, X, Y, and F
    commandArray[i] = strtok_r(commandCopy, " \nXYF", &saveptr);

    //create a for loop to store all tokens of the command within the commandArray
    while(commandArray[i] != NULL) {
      i++;
      commandArray[i] = strtok_r(NULL, " \nXYF", &saveptr); //use NULL for the string part because strtok_r uses saveptr to find where to tokenize
    }

    //populate dst coordinates, but only if the command has x and y coordinates:
    if (commandArray[1] != NULL) {
      dst[0] = atoi(commandArray[1]);
      dst[1] = atoi(commandArray[2]);
    }

    //populate src coordinates with global coordinatess
    src[0] = currCoords[0];
    src[1] = currCoords[1];

    //change variables if certain modes are set:
    if (coordMode == 0) { //if the mode is in relative mode
      //add the destination coordinates to the currrent coordinates
      dst[0] = currCoords[0] + dst[0];
      dst[1] = currCoords[1] + dst[1];
    }

    if (unitMode == 1) { //if the units are set to inches
      //convert the given coordinates to mm
      dst[0] = 25.4*dst[0];
      dst[1] = 25.4*dst[1];
    }

    //check the mode: this if statement handles g-code mode
    if (moveMode == 0) {
      
      //check currCommands for each command type
      if (!strcmp(commandArray[0], "G00")) { //execute if the G00 command is the input
        //The G00 command moves the arm to the X Y coordinates at max speed
        //Remove the X Y coordinates from currCommand and convert them to int

        //Serial.print("In the G00 section\n");
        kinematics(src, dst); 
        //Serial.print("Done!\n");

      }
      
      else if (!strcmp(commandArray[0], "G01")) { //execute if the G01 command is the input
        //The G01 command moves the arm to the X Y coordinates at speed F
        //Remove the X Y coordinates from currCommand and convert them to int
        //Serial.write("In the G01 section\n");

        //Remove the F speed from currCommand and convert it to int
        double speed = atoi(commandArray[3]); 

        //call the kinematics function
        kinematics(src, dst); 
        //Serial.print("Done!\n");
      }
      
      else if (!strcmp(commandArray[0], "G20")) { //execute if G20 is the input
        //The G20 command sets the units to inches
        //set units to Inches
        unitMode = 1;
      }

      else if (!strcmp(commandArray[0], "G21")) { //execute if G21 is the input
        //The G21 command sets the units to millimeters
        //set units to millimeters
        unitMode = 0;
      }

      else if (!strcmp(commandArray[0], "G90")) { //execute if G90 is the input
        //The G90 command sets the coordinates to Absolute mode
        //set coordinate mode to Absolute
        coordMode = 1;
      }

      else if (!strcmp(commandArray[0], "G91")) { //execute if G91 is the input
        //The G91 command sets the coordinates to Relative or Incremental mode
        //set coordinate mode to Relative
        coordMode = 0;
      }

      else if (!strcmp(commandArray[0], "M02")) { //execute if M02 is the input
        //The M02 command ends the program
        Serial.print("Exiting Program\n"); 
        free(commandArray); 
        exit(0);
      }

      else if (!strcmp(currCommand, "M06")) { //execute if M6 is the input
        //The M6 command indicates a writing utensil change within 15 seconds
        Serial.print("in the M06 section\n"); 
        moveThis(); 
        //delay the program by 15 seconds
        //delay(15000); //delay by 15000 ms = 15 s
      }

      else if (!strcmp(currCommand, "M72")) { //execute if M72 is the input
        //The M72 command either returns to home

        //Serial.print("In the M72 section\n");

        //home coordinates
        dst[0] = 1;
        dst[1] = 1; 

        src[0] = currCoords[0];
        src[1] = currCoords[1]; 
        
        kinematics(src, dst); 
        //Serial.print("Done!\n");
      
      }

      else if (!strcmp(commandArray[0], "q")) {//execute if q is the input
        //the q command sets the movement mode to free-draw mode 
        //Serial.print("Entering Free-Hand Mode\n"); 
        moveMode = 1; //sets moveMode to free-draw
      }
    }

    //this else statement handles free-draw mode inputs (when moveMode == 0)
    else {

      int n = 10;

      if (!strcmp(currCommand, "q")) {
        //Serial.print("Entering G-Code Mode\n"); 
        moveMode = 0;
      }

      //use if statements to check for different movement inputs
      if (!strcmp(currCommand, "w")) { //execute if w is the input
        //an input of w moves the arm up for a discrete period
        //Serial.print("w inputted\n"); 

        //change the dst coordinates
        dst[0] = currCoords[0];
        dst[1] = currCoords[1] + 1*n; 

        //call the kinematics function
        kinematics(src, dst); 
      }

      else if (!strcmp(currCommand, "a")) { //execute if a is the input
        //an input of a moves the arm left for 1 inch 

        //change the dst coordinates
        dst[0] = currCoords[0] - 1*n;
        dst[1] = currCoords[1]; 

        //call the kinematics function
        kinematics(src, dst); 
      }

      else if (!strcmp(currCommand, "s")) { //execute if s is the input
        //an input of a moves the arm down for 1 inch (Y coordinate is decreased by 1)

        //change the dst coordinates
        dst[0] = currCoords[0];
        dst[1] = currCoords[1] - 1*n; 

        //call the kinematics function
        kinematics(src, dst); 
      }

      else if (!strcmp(currCommand, "d")) { //execute if d is the input
        //an input of d moves the arm right for 1 inch 

        //change the dst coordinates
        dst[0] = currCoords[0] + 1*n;
        dst[1] = currCoords[1]; 

        //call the kinematics function
        kinematics(src, dst); 
      }

      else if ((!strcmp(currCommand, "wa")) | (!strcmp(currCommand, "aw"))) { //execute if wa or aw is the input
        //an input of aw/wa moves the arm Up and Left for 1 inch (X coordinate is decreased by 0.7071
        // and Y coordinate is increased by 0.7071)

        //change the dst coordinates
        dst[0] = currCoords[0] - 0.7071*n;
        dst[1] = currCoords[1] + 0.7071*n; 

        //call the kinematics function
        kinematics(src, dst); 
      }

      else if ((!strcmp(currCommand, "wd")) | (!strcmp(currCommand, "dw"))) { //execute if wd or dw is the input
        //an input of wd/dw moves the arm Up and right for 1 inch (X and Y coordinates are increased by 1)

        //change the dst coordinates
        dst[0] = currCoords[0] + 0.7071*n;
        dst[1] = currCoords[1] + 0.7071*n; 

        //call the kinematics function
        kinematics(src, dst); 
      }

      else if ((!strcmp(currCommand, "sa")) | (!strcmp(currCommand, "as"))) { //execute if sa or as is the input
        //an input of sa/as moves the arm Down and for 1 inch 

        //change the dst coordinates
        dst[0] = currCoords[0] - 0.7071*n;
        dst[1] = currCoords[1] - 0.7071*n; 

        //call the kinematics function
        kinematics(src, dst); 
      }

      else if ((!strcmp(currCommand, "sd")) | (!strcmp(currCommand, "ds"))) { //execute if sd or ds is the input
        //an input of ds/sd moves the arm Down and right for 1 mm

        //change the dst coordinates
        dst[0] = currCoords[0] + 0.7071*n;
        dst[1] = currCoords[1] - 0.7071*n; 

        //call the kinematics function
        kinematics(src, dst); 
      }
    }
  }

  free(commandArray); 
}
