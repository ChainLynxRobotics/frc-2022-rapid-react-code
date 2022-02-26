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
  private boolean armStatus;
  private double avgDeltaVelocity;
  private double prevVelocity;
  private final static double EXPONENT_WEIGHT = 2;
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
      System.out.println("arm encoder position" + armMotor.getEncoder().getPosition());
  }

  public void raiseArm() {
    
    if (!armStatus) {
      armMotor.set(.2);

      double currentVelocity = armMotor.getEncoder().getVelocity();
      double currentDeltaVelocity = Math.abs(prevVelocity - currentVelocity);

      if (Math.abs(avgDeltaVelocity - currentDeltaVelocity) > (avgDeltaVelocity / 10)) {
        armMotor.set(0);
        armStatus = true;
      }

      prevVelocity = currentVelocity;
      // update EMA
      avgDeltaVelocity = avgDeltaVelocity + (EXPONENT_WEIGHT * (currentDeltaVelocity - avgDeltaVelocity));
    } else {
      armMotor.set(0);
      prevVelocity = 0;
      avgDeltaVelocity = 0;
    }
  }

  public void lowerArm()  {
    if (armStatus) { 

      armMotor.set(-.2);

      double currentVelocity = armMotor.getEncoder().getVelocity();
      double currentDeltaVelocity = Math.abs(prevVelocity - currentVelocity);

      if (Math.abs(avgDeltaVelocity - currentDeltaVelocity) > (avgDeltaVelocity / 10)) {
        armMotor.set(0);
        
        armStatus = false;
      }

      prevVelocity = currentVelocity;
      // update EMA
      avgDeltaVelocity = avgDeltaVelocity + (EXPONENT_WEIGHT * (currentDeltaVelocity - avgDeltaVelocity));

    } else {
      armMotor.set(0);
      prevVelocity = 0;
      avgDeltaVelocity = 0;
    }
  }
  
}
