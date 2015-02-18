/* This file is part of the Razor AHRS Firmware */

//////////////////////////////////////////////////////////////////  
///////// Refer to Paper, Pages 9-10 for explanaiton of //////////
///////// Compass_Heading() function below. //////////////////////
//////////////////////////////////////////////////////////////////

void Compass_Heading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated magnetic field X
  mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch; //AJ - Puts the magnetic field vector in the x into the COG frame
  // Tilt compensated magnetic field Y
  mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll; //AJ - Puts the magnetic field vector in the y into the COG frame like DCM paper
  // Magnetic Heading
  MAG_Heading = atan2(-mag_y, mag_x); //AJ - not sure why they use -mag_y instead of mag_y. I think because arduino defines z-up as positive where as we define z-down as positive.
                                      //This would require us to make mag_y negative I believe. 
}

//////////////////////////////////////////////////////////////////  

