// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.abstractSubsystems.RobotArmBase;

// i know that this might break things but i cant help but have a little fun with abstraction as it also allows for easier tweaking of arm subsystems at competion without breaking code
public class RobotArm extends RobotArmBase {
  /** Creates a new robotArm. */
  
  /*private double upperAngle;
  private double lowerAngle;*/
  
  protected Timer armTimer;
  @Override
  protected void otherConfigs() {
    
    armTimer= new Timer();
    armTimer.start();
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
    
  }

  @Override
  public void moveArm(boolean ArmUp) {
    if (ArmUp) {
      raiseArm();
      if(!armStatus){
        armTimer.reset();
      }
      armStatus = true;
    } else {
      lowerArm();
      if(armStatus){
        armTimer.reset();
      }
      armStatus = false;
    }
    
  }


  
  
}
