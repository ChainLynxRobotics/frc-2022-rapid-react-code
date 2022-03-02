// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.abstractSubsystems.RobotArmBase;

// i know that this might break things but i cant help but have a little fun with abstraction as it also allows for easier tweaking of arm subsystems at competion without breaking code
public class RobotArm extends RobotArmBase {
  /** Creates a new robotArm. */
  private double avgDeltaVelocity;
  private double prevVelocity;
  /*private double upperAngle;
  private double lowerAngle;*/
  
  
  @Override
  protected void otherConfigs() {
    avgDeltaVelocity = 0;
    prevVelocity = 0;   
  }
    
  

  @Override
  public void periodic() {
      //System.out.println("arm encoder position" + armMotor.getEncoder().getPosition());
  }
  @Override
  protected void raiseArm() {
    
    System.out.println("robot arm raise called");
    if(armTimer.get() < 1.5){
      armMotor.set(.3);
      }
      else{
        armMotor.set(0.05);
      }
    /*
    // sorry alex your code doesnt work
    if (!armStatus) {
      System.out.println("robot arm raise running");
      armMotor.set(.2);

      double currentVelocity = armMotor.getEncoder().getVelocity();
      double currentDeltaVelocity = Math.abs(prevVelocity - currentVelocity);

      // update EMA
      avgDeltaVelocity = avgDeltaVelocity + (DriveConstants.ROBOT_ARM_EXPONENT_WEIGHT * (currentDeltaVelocity - avgDeltaVelocity));
      
      if (Math.abs(avgDeltaVelocity - currentDeltaVelocity) > (avgDeltaVelocity / DriveConstants.ROBOT_ARM_DELTA_SENSITIVITY)) {
        System.out.println("robot arm raise stopping");
        armMotor.set(0);
        armStatus = true;
      }

      prevVelocity = currentVelocity;
      
    } else {
      System.out.println("robot arm already raised");
      armMotor.set(0);
      prevVelocity = 0;
      avgDeltaVelocity = 0;
    }
    */
  }
  @Override
  protected void lowerArm()  {
  
    System.out.println("robot arm lower called");
    if(armTimer.get() < 2){
    armMotor.set(-.1);
    }
    else{
      armMotor.set(-
      0.05
      );
    }
    /*
    if (armStatus) { 
      System.out.println("robot arm lower running");

      armMotor.set(-.2);

      double currentVelocity = armMotor.getEncoder().getVelocity();
      double currentDeltaVelocity = Math.abs(prevVelocity - currentVelocity);
      // update EMA
      avgDeltaVelocity = avgDeltaVelocity + (DriveConstants.ROBOT_ARM_EXPONENT_WEIGHT * (currentDeltaVelocity - avgDeltaVelocity));
      
      if (Math.abs(avgDeltaVelocity - currentDeltaVelocity) > (avgDeltaVelocity / DriveConstants.ROBOT_ARM_DELTA_SENSITIVITY)) {
        System.out.println("robot arm lower stopped");
        armMotor.set(0);
        
        armStatus = false;
      }

      prevVelocity = currentVelocity;
      

    } else {
      System.out.println("robot arm already lowered");
      armMotor.set(0);
      prevVelocity = 0;
      avgDeltaVelocity = 0;
    }
    */
  }


  
  
}
