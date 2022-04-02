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
  public void ballHandlerRunning(double dumpSpeed,boolean shootBall, boolean ballHandlerOff, boolean isButtonTwoPressed){
    dumpSpeed = -(dumpSpeed +1)/2;
    if(ballHandlerOff){
      ballHandlerMotor.set(0);
    }
    // i forgot how this works, test to fix it after auto
    else if (shootBall){
      ballHandlerMotor.set(dumpSpeed);
    }
    else if (isButtonTwoPressed) {
      ballHandlerMotor.set(0.4);
    }
    
    else {
      ballHandlerMotor.set(1);
    }

    

    
    SmartDashboard.putBoolean("status/ballHandlerOn", ballHandlerOff);
    SmartDashboard.putNumber("status/ballDumpSpeed", dumpSpeed);
  }

}
