// As it is right now, this code will only work with simple 2D patterns. Some elements will work in
// 3D, but not all will.

// Treat the string from the base to the platform string guide as a leg.

// ###################################################### //
//                    Define Constants                    //
// ###################################################### //
// System origin is at the centre of the platform when it is at its lowest position. This means that the base is -ve Z.
// Hand motor 0 position is when the hand is at the platform. +ve is lowering the hand down (ie. letting out string)

// Siteswap to be Juggled
int siteswap[] = {4};
long throwCounter = 0; // Counter that tracks how many throws have been made
float holdEmptyRatio = 0.5; // Ratio of how long the hand will be full vs. empty (theta in TheMathematicsOfJuggling)
                            // Larger number means longer time holding a ball.
float beatDuration = 0.6; // Seconds
int prevSiteSwap;
float throwHeight;

// Initialize Balls + Hand
const int B = 2; // Number of Balls
const int H = 1; // Number of Hands. This code will be used on one hand, so H will always be 1
bool ballStates[B]; // States for balls to be in (0 = in air, 1 = held)
bool handState = 1; // States for hand to be in (0 = empty, 1 = full). NEED TO START ON 1. If changing, update "time" in GetNextHandPos

// Measured Geometry. DO NOT CHANGE!
float originHeightFromBase = 0.484; // Vertical distance from the base to the origin {m}
float baseRad = 0.51476 / 2; // Radius of base {m}
float platRad = 0.225; // Radius of platform {m}
float baseSmallAngle = 12; // Gamma2 on main sketch {deg}
float platSmallAngle = 12; // Lambda1 on main sketch {deg}
float baseStringGuideToBasezDist = 0.05354; // Distance from base plane to base string guide {m}
float handStringGuideToPlatformZDist = -0.1255; // Distance from centre of platform to string guide {m}
float handStroke = 0.2; // Stroke of hand {m} (max is 0.2)

// Geometry Related to the Hand
float zOffset = 0.01;  // Distance between lowest point of hold and origin (at centre of hand at lowest pos) {m}
                        // This offset is here as a safety to prevent the legs from coming close to their lower limit
float zHold = 0.15;    // Hold z span {m}
float zEmpty = 0.1;   // z span of empty hand (between throw and catch) {m}
float zPath[2] = {zEmpty, zHold}; // z span of next path (hold or empty) {m}

float catchPos[3] = { 0, 0, zOffset + zHold };  // Catch pos {m}
float throwPos[3] = { 0, 0, zOffset + zHold };   // Throw pos {m}

// Framerate of juggle
float frameRate = 100; // Frames/sec
float framesPerBeat = beatDuration * frameRate; // Number of frames in a beat
float msPerFrame = 1000/frameRate; // Number of ms per frame {ms}
long frameCounter; // To keep track of what frame we're up to in the given movement (0 < counter < framesPerBeat)


// ###################################################### //
//                      Set up Motors                     //
// ###################################################### //
#include <AccelStepper.h>

// Initialise motor pins. Order is motor N step, motor N dir, motor N+1 step etc.
byte motorPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 22, 23}; 
byte switches[] = {16, 14, 17, 15, 18, 13}; // Pins of switches for legs 1 -> 6

AccelStepper *motorArray[7] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL};

// General Motor Setup
int stepsPerRev = 400; // As set on Driver
float motMinPulseWidth = 2.5; // Min pulse width to excite motors {us}

// Leg properties
double legTravelPerRotation = 4; // How far the linear actuators move for 1 rotation of the motor (eg. pitch) {mm}
double stepsPer_mm_Travel = stepsPerRev/legTravelPerRotation;

// Leg Motors
int legMotMaxSpeed = 14000; // Hard limit for how fast the motor will go (steps/s). Change with caution.
int legMotMaxAccel = 20000; // Max acceleration (steps/s^2) (15k is proven to work.)
const int legMotMaxSteps = 12000; //17200; // Length of linear actuator stroke (in steps). Could probably go to ~17500. Check.

// Hand Motor
int handMotMaxSpeed = 50000; // Hand motor max speed {steps/sec}
int handMotMaxAccel = 50000; // Hand motor max accel {steps/sec^2}
const int handMotMaxSteps = round(handStroke * 1000 * stepsPer_mm_Travel);

// Switch setup
byte currSwitchStates[6]; // Current switch states
byte lastSwitchStates[6]; // Last switch states
unsigned long lastDebounceTime[6]; // Last time the switch was hit
unsigned long debounceDelay = 20; // ms to wait when debouncing


// ###################################################### //
//       Define arrays that will be populated later       //
// ###################################################### //

// Arrays to hold base and platform nodes. Last element is the hand
float baseNodes[7][3]; // Position vectors of base nodes in base frame
float platNodes[7][3]; // Position vectors of platform nodes in platform frame
float initLegLens[7]; // Lengths of the legs (and string) in their initial (origin) state {m}

// Next position of the hand and platform
float nextHandPos[3]; // The position of the next point along in the path
float nextPlatPos[3]; // Array of positions that the platform will follow (x, y, z positions) {m}
float handPosRelativeToPlatform; // Position of hand along its stroke {m}

// Array that will be used to move motors
int stepNumToMoveMotorsTo[7]; // Position to move motors to to achieve desired platform location {steps}

// Other important vectors to be populated later 
float catchVel[3]; // Catch velocity {m/s}
float throwVel[3]; // Throw velocity {m/s}
float I[3]; // Intersection point of tangents to holdPath at catch and throw. Necessary for finding the platform T and R tensors.
long holdEmptyPathFrames[2]; // [0] is emptyFrames, [1] is holdFrames, to match handState

// Vector for calculating the leg velocities
int lastStepNum[7]; // To record the step position of each motor in the previous frame
float motVelocity[7];


// ###################################################### //
//                        Functions                       //
// ###################################################### //

float ReturnThrowHeight_FillHoldEmptyFrames(int siteSwap){
  // Calculates the number of frames in the holdPath and emptyPath
  // Also returns the height of the next throw
  holdEmptyPathFrames[0] = beatDuration * (1 - holdEmptyRatio) * frameRate;
  holdEmptyPathFrames[1] = beatDuration * (holdEmptyRatio) * frameRate;

  float throwHeight = (9.81 * pow((siteSwap - 2*holdEmptyRatio) * beatDuration, 2)) / 32; // From TheMathematicsOfJuggling pg. 130
  return throwHeight;
}

int ReturnNextThrowSiteSwap(){
  // Grab the next throw's siteswap number from the list

  long modCounter = throwCounter % (sizeof(siteswap) / sizeof(siteswap[0]));
  long siteswapNum = siteswap[modCounter];
  
  return siteswapNum;
}

void SetupMotors(){
  // Initialise all 7 motors and the 6 leg switches. Set max speed/accel.
  for (int mot = 0; mot < 6; mot++){
    motorArray[mot] = new AccelStepper(AccelStepper(1, motorPins[mot * 2], motorPins[mot * 2 + 1]));
    
    // Set limits for motors
    motorArray[mot]->setMaxSpeed(legMotMaxSpeed); // Steps per second
    motorArray[mot]->setAcceleration(legMotMaxAccel); // Steps per second^2
    motorArray[mot]->setMinPulseWidth(motMinPulseWidth); // {us}

    pinMode(switches[mot], INPUT_PULLUP);
  }

  // Set up hand motor (motor 7) independently as it has different demands on it
  motorArray[6]->setMaxSpeed(handMotMaxSpeed);
  motorArray[6]->setAcceleration(handMotMaxAccel);
  motorArray[6]->setMinPulseWidth(motMinPulseWidth);
}

void BuildStewartPlatform(){
  // Build the SP using the given dimensions
  float degToRad = PI/180;

  // Define the angles to the nodes
  float gamma0 = 0; // Offset from horizontal
  float gamma2 = baseSmallAngle; // Angle between close base nodes {deg}
  float gamma1 = 120 - gamma2;   // Angle between far base nodes {deg}

  float lambda1 = platSmallAngle; // Angle between close platform nodes {deg}
  float lambda2 = 120 - lambda1;  // Angle between far platform nodes {deg}
  float lambda0 = (gamma1 - lambda1)/2 + gamma0; // Offset from x axis for platform nodes {deg}

  float baseNodeAngles[6]; // Angles to each of the 6 base nodes {deg}
  float platNodeAngles[6]; // Angles to each of the 6 platform nodes {deg}

  // Define two indices that are used to orient the nodes properly
  float firstAngleIndex;
  float secondAngleIndex;

  for (int node = 0; node < 6; node++){
    firstAngleIndex = floor((float(node) + 2)/2);
    secondAngleIndex = ceil((float(node))/2);

    baseNodeAngles[node] = gamma0 + gamma1 * firstAngleIndex + gamma2 * secondAngleIndex;
    platNodeAngles[node] = lambda0 + lambda1 * firstAngleIndex + lambda2 * secondAngleIndex;

    baseNodes[node][0] = baseRad * cos(baseNodeAngles[node] * degToRad);
    baseNodes[node][1] = baseRad * sin(baseNodeAngles[node] * degToRad);
    baseNodes[node][2] = -originHeightFromBase;

    platNodes[node][0] = platRad * cos(platNodeAngles[node] * degToRad);
    platNodes[node][1] = platRad * sin(platNodeAngles[node] * degToRad);
    platNodes[node][2] = 0;
  }
  
  // Set up hand "nodes"
  baseNodes[6][0] = 0; // Base node of hand part is at the base origin
  baseNodes[6][1] = 0;
  baseNodes[6][2] = -originHeightFromBase + baseStringGuideToBasezDist;

  platNodes[6][0] = 0; // Platform node of hand part is at origin, but offset in the z direction
  platNodes[6][1] = 0;
  platNodes[6][2] = handStringGuideToPlatformZDist;
}

void CalcInitialLegLengths(){
  // Calculate the lengths of the legs {mm} (and the string between the base and the platform string guide) in 
  // the origin position. This length will be effectively set as "home" (ie. step 0)

  float startPos[3] = {0, 0, 0};
  float temp[3]; // To store the vector for the leg under consideration until the length can be calculated

  for (int leg = 0; leg < 7; leg++){
    for (int dim = 0; dim < 3; dim++){
      temp[dim] = startPos[dim] + platNodes[leg][dim] - baseNodes[leg][dim];
    }
    initLegLens[leg] = sqrt( pow(temp[0], 2) + pow(temp[1], 2) + pow(temp[2], 2) );
  }
}

void Debounce(int switchNum){
  if (lastSwitchStates[switchNum] != digitalRead(switches[switchNum])){ // If switch state has changed since last reading
    lastDebounceTime[switchNum] = millis(); // Record the current time
  }

  if ((millis() - lastDebounceTime[switchNum]) > debounceDelay){
    currSwitchStates[switchNum] = digitalRead(switches[switchNum]);
    lastSwitchStates[switchNum] = currSwitchStates[switchNum];
  }
}

void StartUpMotors(){
  // Move all legs slowly downwards until they contact the switches, then extend the legs to the new origin position
  byte distToMoveOnStartup = 10; // Number of steps to move on each iteration of the startup routine. I think this can just be 1, but 10 works...
  int postHomePos = 200 * stepsPer_mm_Travel; // Position to move to after homing
  byte switchSum = 0;
  int homingSpeed = 2000; // Speed to run at when moving to origin position at the end of startup routine {steps/sec}

  while (switchSum < 6){
    for (int sw = 0; sw < 6; sw++){
      Debounce(sw);

      if (lastSwitchStates[sw] == 0){
        motorArray[sw]->move(distToMoveOnStartup);        
      }

      switchSum = lastSwitchStates[0] + lastSwitchStates[1] + lastSwitchStates[2] + 
                  lastSwitchStates[3] + lastSwitchStates[4] + lastSwitchStates[5];

      motorArray[sw]->run();
    }
  }

  for (byte mot = 0; mot < 6; mot++){
    motorArray[mot]->setCurrentPosition(-10); // Establish this position as being the origin (or just off, for a modicum of safety)
    motorArray[mot]->moveTo(postHomePos); // Move to a middle position
    motorArray[mot]->setSpeed(homingSpeed);
    motorArray[mot]->runToPosition();
  }
}

void CalcThrowCatchVel(float height){
  // Calculate the velocity that the ball will be entering/leaving the hand with in the next interaction.
  // ###### THIS WILL NEED REVISITING WHEN DEALING WITH MULTI-HAND SYSTEMS ######

  float velZ = sqrt(2 * 9.81 * height); // Velocity in z direction {m/s}

  float dx = catchPos[0] - throwPos[0]; // x distance ball will travel over as a result of this throw {m}
  float dy = catchPos[1] - throwPos[1]; // y distance ball will travel over as a result of this throw {m}

  float timeOfFlight = 2 * velZ / 9.81; // Time ball is in air (assuming catch/throw at same height) {s}
  
  float velX = dx / timeOfFlight; // Velocity in x direction {m/s}
  float velY = dy / timeOfFlight; // Velocity in y direction {m/s}

  float vels[3] = {velX, velY, velZ};

  for (int i = 0; i < 3; i++){
    int neg = 1;
    if (i == 2){neg = -1;}
    catchVel[i] = vels[i] * neg;
    throwVel[i] = vels[i];
  }
}

void GetNextHandPos(){
  // This function will calculate the path of the next hand movement (either holdPath or emptyPath)
  // Type of path (hold/empty) is determined based on the current state of the hand

  // Start by finding the number of points/frames to use to discretize the path
  long pts = holdEmptyPathFrames[handState]; // Calculate this dynamically in future versions
  float z = zPath[handState]; 

  // Create Bezier curve
  // Start by calculating the location of the 2nd and 3rd control points. This is done by making several assumptions:
  //    1. P1 and P2 are horizontal to each other
  //    2. Hold z span is known and will ALWAYS be met
  //    3. Path is smooth (ie. tangent of bezier will be colinear with catchVel and throwVel)
  float dxCatch = z * catchVel[0]/catchVel[2];
  float dyCatch = z * catchVel[1]/catchVel[2];
  float dxThrow = z * throwVel[0]/throwVel[2];
  float dyThrow = z * throwVel[1]/throwVel[2];

  // Define bezier control points. P0 is catchPos and P3 is throwPos
  float P1[3] = {catchPos[0] - dxCatch, catchPos[1] + dyCatch, catchPos[2] - z};
  float P2[3] = {throwPos[0] - dxThrow, throwPos[1] + dyThrow, throwPos[2] - z}; 
  float P4[3] = {throwPos[0] + dxThrow, throwPos[1] + dyThrow, throwPos[2] + z};
  float P5[3] = {catchPos[0] + dxCatch, catchPos[1] + dyCatch, catchPos[2] + z}; 

  // Populate the path array. I'm sure there's a better way to do this, with array-ifying the control points, but this should work.
  if (handState == 0){ // If Hand is empty
    float time = float(frameCounter) / float(pts) - 1; // Position along path that we're currently at (0 <= time <= 1)
                                                       // Need to subtract 1 here because frameCounter counts the full beat
    for (int dim = 0; dim < 3; dim++) {
    nextHandPos[dim] = throwPos[dim] * pow((1 - time),3) + 
                        P4[dim] * 3.0 * time * pow((1 - time), 2) + 
                        P5[dim] * 3.0 * pow(time, 2) * (1 - time) + 
                        catchPos[dim] * pow(time, 3);
    }
  } 
  else if (handState == 1){ // If Hand is full
    for (int dim = 0; dim < 3; dim++) {
      float time = float(frameCounter) / float(pts); // Position along path that we're currently at (0 <= time <= 1)
      nextHandPos[dim] = catchPos[dim] * pow((1 - time),3) + 
                          P1[dim] * 3.0 * time * pow((1 - time), 2) + 
                          P2[dim] * 3.0 * pow(time, 2) * (1 - time) + 
                          throwPos[dim] * pow(time, 3);
    }
    FindIntersectionOfCatchAndThrowTangents(catchPos, P1, P2, throwPos); // Only needs to be found when the hand is full
  }
}

void FindIntersectionOfCatchAndThrowTangents(float *P0, float *P1, float *P2, float *P3){
  // This function takes in the four control points for the holdPath bezier curve and calculates the position of
  // the intersection point of the tangents to the holdPath at the catch and throw positions (ie. intersection of
  // P0P1 and P2P3). This point is critical for determining the appropriate rotation and translation of the platform
  // (as compared to the ball/hand, which is already determined by the holdPath).

  double grad1 = atan2(P0[0] - P1[0], P0[2] - P1[2]); // Gradient of one side of the curve
  double grad2 = atan2(P3[0] - P2[0], P3[2] - P2[2]); // Gradient of the other side

  if (grad1 != grad2){ // If two lines are not parallel
    float multiplier = ((P3[0]-P2[0]) * (P3[2]-P0[2]) + (P3[2]-P2[2]) * (P0[0] - P3[0])) / 
                       ((P3[0]-P2[0]) * (P1[2] - P0[2]) - (P3[2]-P2[2]) * (P1[0] - P0[0]));
    for (int i = 0; i < 3; i++){
      I[i] = P0[i] + (P1[i]-P0[i]) * multiplier;
    }
  }
  else{ // If two lines are parallel (ie. catchVel and throwVel are parallel)
    I[2] = 100000; // Set z component of I to be really far away
    I[1] = 0; // y component is zero
    I[0] = I[2] * tan(grad1);
  }  
}

float GetNextPlatformPos(long pathLength){
  // Calculates the position of the platform while ball is held. This path is found by adding a certain path to the holdPath. 
  // The "certain path" is a three-point (quadratic) bezier curve. This curve represents the position of the platform relative
  // to the position of the hand/ball. To find the final platform position at point "pt", multiply the "y component" of the bezier
  // by the unit vector that points from the current hand/ball location to the intersection point, I.
  // First, define the three points on this bezier curve. Curve will be drawn on "platPos_rel_handPos" vs "pathLength" axes.
  float P0[2] = {0, 0};
  float P1[2] = {float(pathLength)/2, handStroke};
  float P2[2] = {float(pathLength), 0};
  float P_rel_handPt; // Position of the platform relative to the hand. We only care about the distance along the "I-handPt" axis

  float normalizedPt = float(frameCounter) / pathLength; // How far through the path we currently are (0 <= normalizedPt <= 1)
  float norm_handPt_I = sqrt(pow((I[0] - nextHandPos[0]),2) + pow((I[1] - nextHandPos[1]),2) 
                        + pow((I[2] - nextHandPos[2]),2)); // Length of vector from current point to I
  float e_handPt_I; // Unit vector from current hand/ball pos to I. Only needs to be a scalar though. Just gets overwritten each loop

  P_rel_handPt = P1[1] + pow(1 - normalizedPt, 2) * (P0[1] - P1[1]) + pow(normalizedPt, 2) * (P2[1] - P1[1]); // "Vertical" comp. of Bezier

  // Fill this point's row of the platform path
  for (int j = 0; j < 3; j++){
    e_handPt_I = (I[j] - nextHandPos[j]) / norm_handPt_I;
    nextPlatPos[j] = nextHandPos[j] + e_handPt_I * P_rel_handPt;
  }

  // Return this point's value for the position of the hand relative to the platform
  return P_rel_handPt;
}

double Calc2DRotation(){
  // Calculates the rotation (theta) about the y axis (into/out of the page) for planar motion in the x-z plane. Measured from z axis. CW +ve
  double theta;

  theta = atan2((nextPlatPos[0] - I[0]), (I[2] - nextPlatPos[2]));

  //  // This was an attempt at dealing with the "parallel catch and throw" problem. Is now solved in the "Find I" function. This is backup
  // if (I[0] == I[0]){ // This will be false if I[0] is "NaN"
  //   theta = atan2((nextPlatPos[0] - I[0]), (I[2] - nextPlatPos[2]));
  // } else { // If I[0] is NaN, catchVel and throwVel are parallel. Hence theta is const. and can be found with either vel
  //   theta = atan2(catchVel[0], -catchVel[2]);
  // }

  return theta;
}

void CalcStepsToMoveMotors(double angle, float handPosRelPlat){
  // Calculate how many steps each leg motor will need to turn to move the platform to the desired point
  float temp; // To hold the position changes for the leg motors

  for (int mot = 0; mot < 7; mot++){ // Iterate through all the motors
    // First find the absolute leg lengths at the given point, then subtract the leg's initial length to 
    // get the relative length of this leg at this frame
    temp = CalcLegLengthAtPoint(angle, mot) - initLegLens[mot]; // {m}
    temp = temp * 1000; // Convert to mm
    Serial.print(temp);
    Serial.print(", ");
    Serial.println(round(temp));

    if (mot < 6){ // If a leg motor
      stepNumToMoveMotorsTo[mot] = round(temp);//round(temp * stepsPer_mm_Travel); 

    } else{ // if hand motor
      stepNumToMoveMotorsTo[mot] = round(handPosRelPlat * 1000 + temp); //round(stepsPer_mm_Travel * (handPosRelPlat * 1000 + temp)); 
    }
  } 
}

float CalcLegLengthAtPoint(double theta, int legNum){
  // Calculates the length of leg "legNum" at the given instant
  double R[3][3] = {
                    {cos(theta), 0, sin(theta)},
                    {0, 1, 0},
                    {-sin(theta), 0, cos(theta)}
                  };

  float temp[3];

  for (int dim = 0; dim < 3; dim++) { // 3 dimensions
    temp[dim] = nextPlatPos[dim] + R[dim][0] * platNodes[legNum][0] + R[dim][1] * platNodes[legNum][1] + 
                    R[dim][2] * platNodes[legNum][2] - baseNodes[legNum][dim];
  }

  return sqrt(pow(temp[0], 2) + pow(temp[1], 2) + pow(temp[2], 2));
}

bool LengthSafetyChecker(){
  // Checks the lengths of all the legs against their limits and ensures that there are no over/under extensions
  bool checker = 0; // 0 is fine, 1 is leg is too long/short
  int lengthLims[7] = {legMotMaxSteps, legMotMaxSteps, legMotMaxSteps, 
                         legMotMaxSteps, legMotMaxSteps, legMotMaxSteps, handMotMaxSteps};
  for (byte mot = 0; mot < 7; mot++){
    if (stepNumToMoveMotorsTo[mot] <= lengthLims[mot] && stepNumToMoveMotorsTo[mot] >= 0){
      // Do nothing

    } else if (stepNumToMoveMotorsTo[mot] > lengthLims[mot]){ // If leg is too long
      checker = 1;
      stepNumToMoveMotorsTo[mot] = lengthLims[mot];
      Serial.print("Motor ");
      Serial.print(mot);
      Serial.println("is too long!");

    } else if (stepNumToMoveMotorsTo[mot] < 0){ // If leg is too short
      checker = 1;
      stepNumToMoveMotorsTo[mot] = 0;
      Serial.print("Motor ");
      Serial.print(mot);
      Serial.println("is too short!");
    }
  }

  return checker;
}

void PrimeMotors(){
  // Gets the motors ready to move by setting their target position
  // Called once per frame
  for (byte mot = 0; mot < 7; mot++){
    motorArray[mot]->moveTo(stepNumToMoveMotorsTo[mot]);
  }
}

void MoveMotors(){
  // Moves the motors
  // Call this as often as possible
  for (byte mot = 0; mot < 7; mot++){
    motorArray[mot]->run();
  }
}

void CopyArray(float *src, float *dst, int len){
  for (int i = 0; i < len; i++){
    dst[i] = src[i];
  }
}

void IndexFrameCounter(){
  frameCounter++;
  if (frameCounter == holdEmptyPathFrames[0]){
    // If the last frame of the empty path was just processed, change the status of the hand to be full
    handState = !handState;
  } else if (frameCounter == long(framesPerBeat)){
    // If the last frame of the full beat was processed, change the status of the hand to be empty and reset the counter for the next beat
    handState = !handState;
    frameCounter = 0;
  }
}

void UpdateThrowCatchVel(){
  // Update the throw/catch velocity according to next throw's siteswap. WILL NEED TO FIX THIS TO HAVE TRACKED THROW VS CATCH VELS

  if (prevSiteSwap != ReturnNextThrowSiteSwap()){
    // If siteswap for the next throw is different to the last siteswap, update the throw velocity
    int nextSiteSwap = ReturnNextThrowSiteSwap();
    throwHeight = ReturnThrowHeight_FillHoldEmptyFrames(nextSiteSwap);
    CalcThrowCatchVel(throwHeight);
    prevSiteSwap = nextSiteSwap;
  }

  Serial.print("Siteswap Number: ");
  Serial.println(prevSiteSwap);
  Serial.print("Throw number: ");
  Serial.println(throwCounter);
  Serial.print("Holding Frames: ");
  Serial.println(holdEmptyPathFrames[1]);
  Serial.print("Empty Frames: ");
  Serial.println(holdEmptyPathFrames[0]);
  Serial.print("Throw Height: ");
  Serial.print(throwHeight);
  Serial.print(" m");
  Serial.println();
  Serial.println();

  throwCounter++;
}

void CalcMotVelocities(){
  // Calculates the velocity of the motors between this step and the previous one
  Serial.print("Motor Velocities: ");
  for (int mot = 0; mot < 7; mot++){
    motVelocity[mot] = float(stepNumToMoveMotorsTo[mot] - lastStepNum[mot]) * frameRate;
    Serial.print(motVelocity[mot]);
    Serial.print(", ");

    lastStepNum[mot] = stepNumToMoveMotorsTo[mot];
  }
  Serial.println(" Steps/sec");
}

void PrintMotorPositions(){
  // Simply prints the positions (steps) of all motors at this frame
  Serial.print("Motor Positions: ");
  for (int mot = 0; mot < 7; mot++){
    Serial.print(stepNumToMoveMotorsTo[mot]);
    Serial.print(", ");
  }
}


// ###################################################### //
//                         Setup                          //
// ###################################################### //

void setup() {
  Serial.begin(9600);

  // SetupMotors(); // Initialize the motors + switches
  BuildStewartPlatform(); // Populate the baseNodes and platNodes matrices
  CalcInitialLegLengths(); // Calculate the lengths of the legs when the hand is at the origin

  Serial.print("Initial Leg Lengths: ");
  for (int leg = 0; leg < 6; leg++){
    Serial.print(initLegLens[leg], 3);
    Serial.print(", ");
  }

  // StartupMotors(); // Move through startup sequence
}

// ###################################################### //
//                       Main Loop                        //
// ###################################################### //

void loop() {
  if (frameCounter == 0){ // If starting a new beat
  // Update the throw/catch velocity according to next throw's siteswap
    UpdateThrowCatchVel();
    delay(2000);
  }
  
  // Start each pass by finding the point that the hand will move to
  GetNextHandPos();

  if (handState == 0){ // If hand is empty
    CopyArray(nextHandPos, nextPlatPos, 3); // Platform path will be the same as the hand path
    handPosRelativeToPlatform = 0; // And the relative distance between the hand and platform will be 0
  } 
  else if (handState == 1){ // If hand is full
    // Find the next platform pos and the relative distance between the hand and the platform {m}
    handPosRelativeToPlatform = GetNextPlatformPos(holdEmptyPathFrames[handState]);
  }

  // Calculate the rotation of the platform in the current frame
  double theta = Calc2DRotation();

  CalcStepsToMoveMotors(theta, handPosRelativeToPlatform);

  if (handState == 0){Serial.println("Hand is empty");} 
  else if (handState == 1){Serial.println("Holding Ball");}
  // PrintMotorPositions();
  Serial.println();
  CalcMotVelocities();
  // Serial.print(" Frame number: ");
  // Serial.print(frameCounter);
  // Serial.println();
  // Serial.println();
  // Serial.println(stepNumToMoveMotorsTo[6] / stepsPer_mm_Travel);
  for (int dim = 0; dim < 3; dim++){
    Serial.print(nextHandPos[dim], 3);
    Serial.print(", ");
  }
  Serial.println();  

  IndexFrameCounter(); // Increase the frame count
  delay(500);
}