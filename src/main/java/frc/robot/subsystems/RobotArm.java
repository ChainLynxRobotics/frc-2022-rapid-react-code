// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

public class RobotArm extends SubsystemBase {
  /** Creates a new robotArm. */
  private CANSparkMax armMotor;
  private boolean lastInput;
  private boolean raised;

  /*private double upperAngle;
  private double lowerAngle;*/

  public RobotArm() {
    armMotor = new CANSparkMax(RobotMap.ROBOT_ARM_MOTOR_ID, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kCoast);
    
  }
  
  public void toggleArm(boolean inputStatus){
    if(inputStatus != lastInput && inputStatus == true ){
      try {
        if (raised) {
          lowerArm();
        }else{
          raiseArm();
        }
      } catch (InterruptedException e) {
        System.out.println(e);
      }
    }
    else if(inputStatus != lastInput){
      lastInput = inputStatus;
    }

  }

  public void raiseArm() throws InterruptedException{
    
    armMotor.set(1);
    while(armMotor.getEncoder().getVelocity() > 0){}
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.set(0);
    raised = true;
  }

  public void lowerArm() throws InterruptedException {
    armMotor.setIdleMode(IdleMode.kCoast);
  }
  
}
