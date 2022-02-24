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
  
  

  @Override
  public void periodic() {
      System.out.println("arm encoder position" + armMotor.getEncoder().getPosition());
  }

  public void raiseArm() {
    
    armMotor.set(.1);
    
  }

  public void lowerArm()  {
    armMotor.set(-.1);
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      System.out.println("arm wait didn't work");
    }
    while(armMotor.getEncoder().getVelocity() < 0){
      //System.out.println("armlowering");
    }
    armMotor.set(0);
    
  }
  
}
