// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// note please ask whoever the operator ends up being what they want to press for intake
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  private WPI_VictorSPX intakeMotor;
  
  public Intake() {
    intakeMotor = new WPI_VictorSPX(RobotMap.INTAKE_MOTOR_ID);
    
  }

      
  public void intakeRunning(double inputSpeed, boolean intakeOn){
    if(intakeOn == false){
      intakeMotor.set(0);
    } 
    else if( inputSpeed > 0.05 ){
      intakeMotor.set(-1);
    }
    else{
      intakeMotor.set(1);
    }
    
  }

}
