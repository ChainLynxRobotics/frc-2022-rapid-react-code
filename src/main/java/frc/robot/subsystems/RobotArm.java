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
  
  /*private double upperAngle;
  private double lowerAngle;*/

  public RobotArm() {
    armMotor = new CANSparkMax(RobotMap.ROBOT_ARM_MOTOR_ID, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    
  }
  
  

  

  public void raiseArm() {
    
    armMotor.set(.05);
    while(armMotor.getEncoder().getVelocity() > 0){}
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.set(0);
    
  }

  public void lowerArm()  {
    armMotor.setIdleMode(IdleMode.kCoast);
  }
  
}
