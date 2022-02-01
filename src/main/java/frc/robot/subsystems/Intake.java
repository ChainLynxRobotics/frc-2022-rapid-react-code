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
  private boolean currentIntakeStatus;
  public Intake() {
    intakeMotor = new WPI_VictorSPX(5);

    currentIntakeStatus = false;
  }

  
  public void intakeRunning(boolean intakeOn){
    if (intakeOn != currentIntakeStatus){
      if(intakeOn == true){
        intakeMotor.set(1.0);
      }
      else{
        intakeMotor.set(0);
      }
      currentIntakeStatus = intakeOn;
    }
  }

}
