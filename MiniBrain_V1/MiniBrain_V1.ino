// Define constants //

const int pts = 50;  // Number of points to discretize the holdPath/emptyPath with

float zOffset = 0.1;  // Distance between lowest point of hold and origin (at centre of hand at lowest pos) {m}
float zHold = 0.2;    // Hold z span {m}
float zEmpty = 0.1;  // z span of empty hand (between throw and catch) {m}
float throwHeight = 0.5; // Throw height from throw/catch position (zHold + zOffset) {m}
float handStroke = 0.1; // Stroke of hand {m}

float catchPos[3][1] = { -0.1, 0, zOffset + zHold };  // Catch pos {m}
float throwPos[3][1] = { 0.1, 0, zOffset + zHold };   // Throw pos {m}

float baseRad = 1; // Radius of base {m}
float platRad = 1; // Radius of platform {m}
float baseSmallAngle = 12; // Gamma2 on main sketch {deg}
float platSmallAngle = 12; // Lambda1 on main sketch {deg}

// Initialize vectors/arrays to be populated later // 

float catchVel[3][1]; // Catch velocity {m/s}
float throwVel[3][1]; // Throw velocity {m/s}
float I[3][1]; // Intersection point of tangents to holdPath at catch and throw. Necessary for finding the platform T and R tensors.

float holdPath[pts][3];  // Make array of positions that the ball/hand will follow while the ball is held (x, y, z positions) {m}
float emptyPath[pts][3];  // Make array of positions the platform will follow while ball is in the air (x, y, z positions) {m}
float platPath[pts][3]; // Array of positions that the platform will follow while ball is held (x, y, z positions) {m}

// Arrays to hold base and platform nodes
float baseNodes[6][3]; // Position vectors of base nodes in base frame
float platNodes[6][3]; // Position vectors of platform nodes in platform frame

// Arrays that will be used to move motors
float handPosRelativeToPlatform[pts][1]; // Position of hand along its stroke {m}
float legLengths[pts][6]; // Lengths of the six legs {m}

void CalcHoldPath(float *holdpath, float *legLens, int pts, float *catchP, float *throwP, float *catchV, float *throwV, float zhold){
  // Create Bezier curve
  // Start by calculating the location of the 2nd and 3rd control points. This is done by making several assumptions:
  //    1. P1 and P2 are horizontal to each other
  //    2. Hold z span is known and will ALWAYS be met
  //    3. Path is smooth (ie. tangent of bezier will be colinear with catchVel and throwVel)

  float dxCatch = zhold * catchV[0]/catchV[2];
  float dyCatch = zhold * catchV[1]/catchV[2];
  float dxThrow = zhold * throwV[0]/throwV[2];
  float dyThrow = zhold * throwV[1]/throwV[2];
  float handPt[3]; // Current location of hand/ball

  float P1[3] = {catchP[0] - dxCatch, catchP[1] + dyCatch, catchP[2] - zhold}; 
  float P2[3] = {throwP[0] - dxThrow, throwP[1] + dyThrow, throwP[2] - zhold}; 

  float time[pts]; // Position along the bezier (0 <= time <= 1)

  // Fill "time" vector. Will be a vector with "pts" rows that spans from 0 <= time <= 1.
  for (int i = 0; i < pts; i++) {
    time[i] = 0 + i * 1.0/(pts-1); // Calculate position along bezier that we're currently at

    // Populate the holdPath array.
    for (int j = 0; j < 3; j++) {
      holdpath[i*3 + j] = catchP[j] * pow((1 - time[i]),3) + 
                          P1[j] * 3.0 * time[i] * pow((1 - time[i]), 2) + 
                          P2[j] * 3.0 * pow(time[i], 2) * (1 - time[i]) + 
                          throwP[j] * pow(time[i], 3);
      handPt[j] = holdpath[i*3 + j];
    }

    // Calculate the platform path
    CalcPlatformPath(*platPath, *handPosRelativeToPlatform, *I, handPt, pts, i);

    // Calculate platform rotation for this point
    double theta = Calc2DRotation(handPt);

    // Calculate leg lengths for this point !
    for (int leg = 0; leg < 6; leg++){
      legLens[i*6 + leg] = CalcLegLengthAtPoint(handPt, theta, leg);
    }
    // float legLengths[6] = CalcLegLengthsAtPoint(handPt, theta);
    
  }

  // Calculate the intersection point of the catch and throw tangents for this holdPath
  FindIntersectionOfCatchAndThrowTangents(*I, catchP, P1, P2, throwP);
}

double Calc2DRotation(float *currentPt){
  // Calculates the rotation (theta) about the y axis (into/out of the page) for planar motion in the x-z plane
  return atan2((currentPt[0] - I[0][0]), (I[0][0] - currentPt[2]));
}

float CalcLegLengthAtPoint(float *platPos, double theta, int legNum){
  double R[3][3] = {
                    {cos(theta), 0, sin(theta)},
                    {0, 1, 0},
                    {-sin(theta), 0, cos(theta)}
                  };

  float temp[3];

  for (int dim = 0; dim < 3; dim++) { // 3 dimensions
    temp[dim] = platPos[dim] + R[dim][0] * platNodes[legNum][0] + R[dim][1] * platNodes[legNum][1] + 
                    R[dim][2] * platNodes[legNum][2] - baseNodes[legNum][dim];
  }

  return sqrt(pow(temp[0], 2) + pow(temp[1], 2) + pow(temp[2], 2));
}

void BuildStewartPlatform(float *baseNodes, float *platNodes){
  // Build the SP using the given dimensions

  float degToRad = PI/180;

  // Define the angles to the nodes
  float gamma0 = 0; // Offset from horizontal
  float gamma2 = baseSmallAngle; // Angle between close base nodes {deg}
  float gamma1 = 120 - gamma2; // Angle between far base nodes {deg}

  float lambda1 = platSmallAngle; // Angle between close platform nodes {deg}
  float lambda2 = 120 - lambda1; // Angle between far platform nodes {deg}
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

    baseNodes[node*3 + 0] = baseRad * cos(baseNodeAngles[node] * degToRad);
    baseNodes[node*3 + 1] = baseRad * sin(baseNodeAngles[node] * degToRad);
    baseNodes[node*3 + 2] = 0;

    platNodes[node*3 + 0] = platRad * cos(platNodeAngles[node] * degToRad);
    platNodes[node*3 + 1] = platRad * sin(platNodeAngles[node] * degToRad);
    platNodes[node*3 + 2] = 0;
  }
}

void CalcEmptyPath(float *array, int pts, float *catchP, float *throwP, float *catchV, float *throwV, float zempty){
  // Create Bezier curve
  // Start by calculating the location of the 4th and 5th control points. Numbering continues from that used in holdPath
  // This is done by making the same assumptions as in the holdPath control point calculations:
  //    1. P4 and P5 are horizontal to each other
  //    2. Empty z span is known and will ALWAYS be met
  //    3. Path is smooth (ie. tangent of bezier will be colinear with catchVel and throwVel)

  float dxCatch = zempty * catchV[0]/catchV[2];
  float dyCatch = zempty * catchV[1]/catchV[2];
  float dxThrow = zempty * throwV[0]/throwV[2];
  float dyThrow = zempty * throwV[1]/throwV[2];

  float P4[3] = {throwP[0] + dxThrow, throwP[1] + dyThrow, throwP[2] + zempty};
  float P5[3] = {catchP[0] + dxCatch, catchP[1] + dyCatch, catchP[2] + zempty};  

  float time[pts]; // Position along the bezier (0 <= time <= 1)

  // Fill "time" vector. Will be a vector with "pts" rows that spans from 0 <= time <= 1.
  for (int i = 0; i < pts; i++) {
    time[i] = 0 + i * 1.0/(pts-1); // Calculate position along bezier that we're currently at

    // Populate the emptyPath array.
    for (int j = 0; j < 3; j++) {
      array[i*3 + j] = throwP[j] * pow((1 - time[i]),3) + 
                          P4[j] * 3.0 * time[i] * pow((1 - time[i]), 2) + 
                          P5[j] * 3.0 * pow(time[i], 2) * (1 - time[i]) + 
                          catchP[j] * pow(time[i], 3);
    }
  }
}

void FindIntersectionOfCatchAndThrowTangents(float *intersectionPt, float *P0, float *P1, float *P2, float *P3){
  // This function takes in the four control points for the holdPath bezier curve and calculates the position of
  // the intersection point of the tangents to the holdPath at the catch and throw positions (ie. intersection of
  // P0P1 and P2P3). This point is critical for determining the appropriate rotation and translation of the platform
  // (as compared to the ball/hand, which is already determined by the holdPath).

  float multiplier = ((P3[0]-P2[0]) * (P3[2]-P0[2]) + (P3[2]-P2[2]) * (P0[0] - P3[0])) / 
                      ( (P3[0]-P2[0]) * (P1[2] - P0[2]) - (P3[2]-P2[2]) * (P1[0] - P0[0]) );
  for (int i = 0; i < 3; i++){
    intersectionPt[i] = P0[i] + (P1[i]-P0[i]) * multiplier;
  }
}

void CalcPlatformPath(float *platformPath, float *handpos, float *intersectionPt, float *handPt, float pathLength, int pt){
  // Calculates the position of the platform while ball is held. This path is found by adding a certain path to the holdPath. 
  // The "certain path" is a three-point (quadratic) bezier curve. This curve represents the position of the platform relative
  // to the position of the hand/ball. To find the final platform position at point "pt", multiply the "y component" of the bezier
  // by the unit vector that points from the current hand/ball location to the intersection point, I.
  // First, define the three points on this bezier curve. Curve will be drawn on "platPos_rel_handPos" vs "pathLength" axes.
  float P0[2] = {0, 0};
  float P1[2] = {pathLength/2, handStroke};
  float P2[2] = {pathLength, 0};
  float P_rel_handPt; // Position of the platform relative to the hand. We only care about the distance along the "I-handPt" axis

  float normalizedPt = pt / pathLength; // How far through the path we currently are (0 <= normalizedPt <= 1)
  float norm_handPt_I = sqrt(pow((intersectionPt[0] - handPt[0]),2) + pow((intersectionPt[1] - handPt[1]),2) 
                        + pow((intersectionPt[2] - handPt[2]),2)); // Length of vector from current point to I
  float e_handPt_I[3]; // Unit vector from current hand/ball pos to I

  P_rel_handPt = P1[1] + pow(1 - normalizedPt, 2) * (P0[1] - P1[1]) + pow(normalizedPt, 2) * (P2[1] - P1[1]);

  // Fill this point's value for the position of the hand relative to the platform
  handpos[pt] = -P_rel_handPt;

  // Fill this point's row of the platform path
  for (int j = 0; j < 3; j++){
    e_handPt_I[j] = (intersectionPt[j] - handPt[j]) / norm_handPt_I;
    platformPath[pt*3 + j] = handPt[j] + e_handPt_I[j] * P_rel_handPt;
  }
}

void CalcThrowCatchVel(float *catchV, float *throwV, float *catchP, float *throwP, float height){
  // Calculate the velocity that the ball will be entering/leaving the hand with in the next interaction.
  // ###### THIS WILL NEED REVISITING WHEN DEALING WITH MULTI-HAND SYSTEMS ######

  float velZ = sqrt(2 * 9.81 * height); // Velocity in z direction {m/s}

  float dx = catchP[0] - throwP[0]; // x distance ball will travel over as a result of this throw {m}
  float dy = catchP[1] - throwP[1]; // y distance ball will travel over as a result of this throw {m}

  float timeOfFlight = 2 * velZ / 9.81; // Time ball is in air (assuming catch/throw at same height) {s}
  
  float velX = dx / timeOfFlight; // Velocity in x direction {m/s}
  float velY = dy / timeOfFlight; // Velocity in y direction {m/s}

  float vels[3] = {velX, velY, velZ};

  for (int i = 0; i < 3; i++){
    int neg = 1;
    if (i == 2){neg = -1;}
    catchV[i] = vels[i] * neg;
    throwV[i] = vels[i];
  }
}

void setup() {
  Serial.begin(9600);

  BuildStewartPlatform(*baseNodes, *platNodes); // Populate the baseNodes and platNodes matrices

  CalcThrowCatchVel(*catchVel, *throwVel, *catchPos, *throwPos, throwHeight); // Calculate catch/throw velocities
  CalcHoldPath(*holdPath, *legLengths, pts, *catchPos, *throwPos, *catchVel, *throwVel, zHold);
  // CalcEmptyPath(*emptyPath, pts, *catchPos, *throwPos, *catchVel, *throwVel, zEmpty);


  Serial.println();
  for (int pt = 0; pt < pts; pt++){
    for (int i = 0; i < 6; i++) {
      Serial.print(legLengths[pt][i]);
      Serial.print(", ");
    }
    Serial.println();
  }
}

void loop() {
  // checkVelocities();
  // CalcHoldPath(*holdPath, pts, *catchPos, *throwPos, *catchVel, *throwVel, zHold);
  // checkPaths();

  // for (int pt = 0; pt < pts; pt++){
  //   for (int i = 0; i < 3; i++) {
  //     Serial.print(platPath[pt][i]);
  //     Serial.print(", ");
  //   }
  //   Serial.println();
  // }
}

void checkVelocities(){
  for (int i = 0; i < 3; i++) {
    Serial.print(catchVel[i][0]);
    // Serial.print(catchVel[i][0]);
    Serial.print(", ");
  }
  Serial.println();
  delay(1000);
}

void checkPaths(){
  for (int i = 0; i < pts; i++) {
    for (int j = 0; j < 3; j++) {
      // Serial.print(emptyPath[i][j],4);
      Serial.print(holdPath[i][j],4);
      Serial.print(", ");
    }
    Serial.println();
  }
  Serial.println();
  delay(1000);
}