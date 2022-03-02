// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// note please ask whoever the operator ends up being what they want to press for intake
package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

public class BallHandler extends SubsystemBase {
  /** Creates a new Intake. */
  
  private WPI_VictorSPX ballHandlerMotor;
  
  public BallHandler() {
    ballHandlerMotor = new WPI_VictorSPX(RobotMap.BALLHANDLER_MOTOR_ID);
    
  }

      
  public void ballHandlerRunning(double inputSpeed, boolean ballHandlerOff){
    
    if(ballHandlerOff == true){
      ballHandlerMotor.set(0);
    } 
    // i forgot how this works, test to fix it after auto
    else if( inputSpeed > 0.2 ){
      ballHandlerMotor.set(.7);
    }
    else if (inputSpeed < -0.2){
      ballHandlerMotor.set(.25);
    }
    else{
      ballHandlerMotor.set(-.5);
    }
    
    SmartDashboard.putBoolean("status/ballHandlerOn", ballHandlerOff);
  }

}
