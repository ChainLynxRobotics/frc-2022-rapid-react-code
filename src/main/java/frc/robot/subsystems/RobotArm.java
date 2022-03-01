// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;

public class RobotArm extends SubsystemBase {
  /** Creates a new robotArm. */
  private CANSparkMax armMotor;
  private boolean armStatus;
  private double avgDeltaVelocity;
  private double prevVelocity;
  /*private double upperAngle;
  private double lowerAngle;*/
  
  public RobotArm() {
    armMotor = new CANSparkMax(RobotMap.ROBOT_ARM_MOTOR_ID, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    
    armStatus = true;

    avgDeltaVelocity = 0;
    prevVelocity = 0;
  }
  
  

  @Override
  public void periodic() {
      //System.out.println("arm encoder position" + armMotor.getEncoder().getPosition());
  }

  public void raiseArm() {

    System.out.println("robot arm raise called");
    
    if (!armStatus) {
      System.out.println("robot arm raise running");
      armMotor.set(.2);

      double currentVelocity = armMotor.getEncoder().getVelocity();
      double currentDeltaVelocity = Math.abs(prevVelocity - currentVelocity);

      // update EMA
      avgDeltaVelocity = avgDeltaVelocity + (DriveConstants.ROBOT_ARM_EXPONENT_WEIGHT * (currentDeltaVelocity - avgDeltaVelocity));
      
      if (Math.abs(avgDeltaVelocity - currentDeltaVelocity) > (avgDeltaVelocity / DriveConstants.ROBOT_ARM_DELTA_SENSITIVITY)) {
        System.out.println("robot arm raise stopping");
        armMotor.set(.1);
        armStatus = true;
      }

      prevVelocity = currentVelocity;
      
    } else {
      System.out.println("robot arm already raised");
      armMotor.set(.1);
      prevVelocity = 0;
      avgDeltaVelocity = 0;
    }
  }

  public void lowerArm()  {

    System.out.println("robot arm lower called");
    if (armStatus) { 
      System.out.println("robot arm lower running");

      armMotor.set(-.2);

      double currentVelocity = armMotor.getEncoder().getVelocity();
      double currentDeltaVelocity = Math.abs(prevVelocity - currentVelocity);
      // update EMA
      avgDeltaVelocity = avgDeltaVelocity + (DriveConstants.ROBOT_ARM_EXPONENT_WEIGHT * (currentDeltaVelocity - avgDeltaVelocity));
      
      if (Math.abs(avgDeltaVelocity - currentDeltaVelocity) > (avgDeltaVelocity / DriveConstants.ROBOT_ARM_DELTA_SENSITIVITY)) {
        System.out.println("robot arm lower stopped");
        armMotor.set(-0.1);
        
        armStatus = false;
      }

      prevVelocity = currentVelocity;
      

    } else {
      System.out.println("robot arm already lowered");
      armMotor.set(-.1);
      prevVelocity = 0;
      avgDeltaVelocity = 0;
    }
  }

  public void moveArm(boolean ArmUp) {
    
    if (ArmUp) {
      raiseArm();
    } else {
      lowerArm();
    }
  }
  
}
