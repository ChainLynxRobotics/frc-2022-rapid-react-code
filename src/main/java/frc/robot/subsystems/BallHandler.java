// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// note please ask whoever the operator ends up being what they want to press for intake
package frc.robot.subsystems;



import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

public class BallHandler extends SubsystemBase {
  /** Creates a new Intake. */
  
  private PWMTalonSRX ballHandlerMotor;
  
  public BallHandler() {
    ballHandlerMotor = new PWMTalonSRX(RobotMap.BALLHANDLER_MOTOR_ID);
    
  }

      
  public void ballHandlerRunning(double inputSpeed, boolean ballHandlerOff){
    
    if(ballHandlerOff == true){
      ballHandlerMotor.set(0);
    } 
    else if( inputSpeed > 0.05 ){
      ballHandlerMotor.set(-.7);
    }
    else if (inputSpeed < -0.05){
      ballHandlerMotor.set(1);
    }
    else{
      ballHandlerMotor.set(.5);
    }
    
    SmartDashboard.putBoolean("status/ballHandlerOn", ballHandlerOff);
  }

}
