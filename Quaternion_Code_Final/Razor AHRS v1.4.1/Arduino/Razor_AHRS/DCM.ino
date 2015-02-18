/* This file is part of the Razor AHRS Firmware */

/**************************************************/

//////////////////////////////////////////////////////////////////
/// The "Drift_coorection function" is discussed on pages ////////
/// 11-13 of the provided paper. /////////////////////////////////
//////////////////////////////////////////////////////////////////

void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight; 
  float Rz[3];
  
  Rz[0] = 2*q[0]*q[2] - 2*q[1]*q[3];
  Rz[1] = 2*q[0]*q[3] + 2*q[1]*q[2];
  Rz[2] = -q[0]*q[0] - q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //AJ - costrains 1 - 2*abs(1 - Accel_magnitude) between 0 and 1. 

  //////////////////////////////////////////////////////////////////
  /// PI Feedback control setup ////////////////////////////////////
  //////////////////////////////////////////////////////////////////
  
  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],Rz); //adjust the ground of reference
  
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight); //AJ - multiplies the errorRollPitch vector by Kp*Accel_weight and saves it to Omega_P[0]
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight); //AJ - multiplies the errorRollPitch vector by Ki*Accel_weight and saves it to Scaled_Omega_I[0]
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I); //AJ - adds the Scaled_Omega_I to Omega_I and save it to Omega_I. 
  
  //////////////////////////////////////////////////////////////////
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(MAG_Heading); //AJ - This is COGX in the DCM Paper
  mag_heading_y = sin(MAG_Heading); //AJ - This is COGY in the DCM paper
  errorCourse=((q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])*mag_heading_y) - ((2*q[0]*q[1] + 2*q[2]*q[3])*mag_heading_x); //Calculating YAW error, AJ - This is the Z component of the cross product or Rx with COG vector
                                                                                   
  //////////////////////////////////////////////////////////////////
  /// PI Feedback control setup ////////////////////////////////////
  //////////////////////////////////////////////////////////////////
  
  Vector_Scale(errorYaw,Rz,errorCourse); //Applies the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
  
  //////////////////////////////////////////////////////////////////
  
}

  
// quat init

////////////////////////////////////////////////////////////////// 
/// Quaternion_update function is explained further on pages  //// 
/// 14-15 of the provided paper. /////////////////////////////////
////////////////////////////////////////////////////////////////// 
void Quaternion_update(void) 
{
  float Step_1[4];
  float Temp_Gyro[3];
  float Temp_Omega[3];
  float Temp_q[4];
  float n=0;
  float sigma[4][4];
  
  ////////////////////////////////////////////////////////////////// 
  /// Below is just a scaled value of the gyro outputs. The ////////
  /// scaling value was not altered for the quaternion /////////////
  /// implemantation. //////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////   
  Gyro_Vector[0]=GYRO_SCALED_RAD(gyro[0]); //gyro x roll
  Gyro_Vector[1]=GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
  Gyro_Vector[2]=GYRO_SCALED_RAD(gyro[2]); //gyro z yaw
  ////////////////////////////////////////////////////////////////// 
  
  
  ////////////////////////////////////////////////////////////////// 
  /// This just saves the accelerometer values to a seprate ////////
  /// variable. ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////// 
  Accel_Vector[0]=accel[0];
  Accel_Vector[1]=accel[1];
  Accel_Vector[2]=accel[2];
  ////////////////////////////////////////////////////////////////// 
  
  
  ////////////////////////////////////////////////////////////////// 
  /// This is the final part of forming the PI feedback control. ///
  /// See page 13 of the provided document for more explanation ////
  //////////////////////////////////////////////////////////////////   
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term 
  ////////////////////////////////////////////////////////////////// 
  
  ////////////////////////////////////////////////////////////////// 
  /// The purpose of the below if statement is to see if any /////// 
  /// any gyro correction was needed for the quaternion update. //// 
  /// If not, than it uses the gyro vector by itself. Else, it ///// 
  /// uses the corrected gyro vector. //////////////////////////////
  ////////////////////////////////////////////////////////////////// 
  
#if DEBUG__NO_DRIFT_CORRECTION == true // Do not use drift correction
  
  Vector_Scale(Temp_Gyro, Gyro_Vector, 0.5*G_Dt);
  
  sigma[0][0]=0;
  sigma[0][1]=Temp_Gyro[2];
  sigma[0][2]=-Temp_Gyro[1];
  sigma[0][3]=Temp_Gyro[0];
  sigma[1][0]=-Temp_Gyro[2];
  sigma[1][1]=0;
  sigma[1][2]=Temp_Gyro[0];
  sigma[1][3]=Temp_Gyro[1];
  sigma[2][0]=Temp_Gyro[1];
  sigma[2][1]=-Temp_Gyro[0];
  sigma[2][2]=0;
  sigma[2][3]=Temp_Gyro[2];
  sigma[3][0]=-Temp_Gyro[0];
  sigma[3][1]=-Temp_Gyro[1];
  sigma[3][2]=-Temp_Gyro[2];
  sigma[3][3]=0;
  
  
  Matrix_Vector_Multiply1(sigma, q, Step_1);
  Vector_Add1(Temp_q, q, Step_1);


#else // Use drift correction
  
  Vector_Scale(Temp_Omega, Omega_Vector, 0.5*G_Dt);
  
  sigma[0][0]=0;
  sigma[0][1]=Temp_Omega[2];
  sigma[0][2]=-Temp_Omega[1];
  sigma[0][3]=Temp_Omega[0];
  sigma[1][0]=-Temp_Omega[2];
  sigma[1][1]=0;
  sigma[1][2]=Temp_Omega[0];
  sigma[1][3]=Temp_Omega[1];
  sigma[2][0]=Temp_Omega[1];
  sigma[2][1]=-Temp_Omega[0];
  sigma[2][2]=0;
  sigma[2][3]=Temp_Omega[2];
  sigma[3][0]=-Temp_Omega[0];
  sigma[3][1]=-Temp_Omega[1];
  sigma[3][2]=-Temp_Omega[2];
  sigma[3][3]=0;
  
  Matrix_Vector_Multiply1(sigma, q, Step_1);
  Vector_Add1(Temp_q, q, Step_1);

  
#endif
  
  ////////////////////////////////////////////////////////////////// 
  /// Renormalization of q to remove numerical errors. ///////////// 
  ////////////////////////////////////////////////////////////////// 
  n=sqrt(Vector_Dot_Product1(Temp_q,Temp_q));
  Vector_Scale1(q,Temp_q,1/n);
  ////////////////////////////////////////////////////////////////// 
  
}

////////////////////////////////////////////////////////////////// 

////////////////////////////////////////////////////////////////// 
/// The "Euler_angles" function is discussed on pages 15-16 of ///
/// the provided paper. //////////////////////////////////////////
////////////////////////////////////////////////////////////////// 

void Euler_angles(void)
{
  pitch = -asin(2*q[0]*q[2] - 2*q[1]*q[3]);
  roll = atan2(2*q[0]*q[3] + 2*q[1]*q[2],-q[0]*q[0] - q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  yaw = atan2(2*q[0]*q[1] + 2*q[2]*q[3],q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
}

////////////////////////////////////////////////////////////////// 

