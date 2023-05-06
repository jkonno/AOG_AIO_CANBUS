void calcSteeringPID(void)
{
  //Proportional only
  pValue = steerSettings.Kp * steerAngleError;
  pwmDrive = (int16_t)pValue;

  errorAbs = abs(steerAngleError);
  int16_t newMax = 0;

  if (errorAbs < LOW_HIGH_DEGREES)
  {
    newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
  }
  else newMax = steerSettings.highPWM;

  //add min throttle factor so no delay from motor resistance.
  if (pwmDrive < 0 ) pwmDrive -= steerSettings.minPWM;
  else if (pwmDrive > 0 ) pwmDrive += steerSettings.minPWM;

  //Serial.print(newMax); //The actual steering angle in degrees
  //Serial.print(",");

  //limit the pwm drive
  if (pwmDrive > newMax) pwmDrive = newMax;
  if (pwmDrive < -newMax) pwmDrive = -newMax;

  if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

  if (steerConfig.IsDanfoss)
  {
    //Serial.print("pwmDrive in: ");
    //Serial.println(pwmDrive);

    // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
    // Danfoss: PWM 50% On = Center Position
    // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
    pwmDrive = (constrain(pwmDrive, -250, 250));

    //Serial.print("pwmDrive constrained: ");
    //Serial.println(pwmDrive);

    // Calculations below make sure pwmDrive values are between 65 and 190
    // This means they are always positive, so in motorDrive, no need to check for
    // steerConfig.isDanfoss anymore
    pwmDrive = pwmDrive >> 2; // Devide by 4
    pwmDrive += 128;          // add Center Pos.

    // pwmDrive now lies in the range [65 ... 190], which would be great for an ideal opamp
    // However the TLC081IP is not ideal. Approximating from fig 4, 5 TI datasheet, @Vdd=12v, T=@40Celcius, 0 current
    // Voh=11.08 volts, Vol=0.185v
    // (11.08/12)*255=235.45
    // (0.185/12)*255=3.93
    pwmDrive = (map(pwmDrive, 4, 235, 0, 255));
    // output now lies in the range [67 ... 205], the center position is now 136

    //Serial.print("pwmDrive: ");
    //Serial.println(pwmDrive);
  }
}

//#########################################################################################

void motorDrive(void) 
  {
    //EDIT FOR CAN VALVE START
    // Used with Isobus message CAN valve
    if (pwmDrive > 0)
    {
      flowCommand.buf[2]=0x01;  //set the correct direction
    }
    else if (pwmDrive < 0) 
    {
      flowCommand.buf[2]=0x02;
      pwmDrive = -pwmDrive;
    } else
    {
      flowCommand.buf[2]=0x00;
      pwmDrive = 0;
    }
    //write out the 0 to 255 value
    flowCommand.buf[0]=(uint8_t)pwmDrive;
    writeToBus(flowCommand);
    pwmDisplay = pwmDrive;
    //EDIT FOR CAN VALVE END
  }
