// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.abstractSubsystems.RobotArmBase;


// i know that this might break things but i cant help but have a little fun with abstraction as it also allows for easier tweaking of arm subsystems at competion without breaking code
public class RobotArm extends RobotArmBase {

private final  PIDController pid = new PIDController(0.6, 0, 0);
  /** Creates a new robotArm. */
  
  /*private double upperAngle;
  private double lowerAngle;*/
  
  protected Timer armTimer;
  public double setpoint;
  public double encoderPosition = 0;
  private final Encoder encoder = (Encoder) armMotor.getEncoder();
  @Override
  protected void otherConfigs() {
    
    armTimer= new Timer();
    armTimer.start();
  }
    
  @Override
  protected void raiseArm() {
    
    System.out.println("robot arm raise called");
    setpoint = 0;
    armMotor.set(MathUtil.clamp(pid.calculate(encoder.getDistance(), setpoint), -0.3, 3));
    
  }
  @Override
  protected void lowerArm()  {
  
    System.out.println("robot arm lower called");
    setpoint = 75;
    armMotor.set( MathUtil.clamp(pid.calculate(encoder.getDistance(), setpoint), -0.3, 0.3));
    
    
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
