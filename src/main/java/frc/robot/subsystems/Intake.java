// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// note please ask whoever the operator ends up being what they want to press for intake
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  private WPI_VictorSPX intakeMotor;
  
  public Intake() {
    intakeMotor = new WPI_VictorSPX(5);

    
  }

      
  public void intakeRunning(double inputSpeed){
    if( inputSpeed > 0.05 ){
      intakeMotor.set(1);
    }
    else if( inputSpeed < 0.05){
      intakeMotor.set(-1);
    }
    else{
      intakeMotor.set(0);
    }
  }

}
