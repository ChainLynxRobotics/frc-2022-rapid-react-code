// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.abstractSubsystems.RobotArmBase;
// i know that this might break things but i cant help but have a little fun with abstraction as it also allows for easier tweaking of arm subsystems at competion without breaking code
public class RobotArmPID extends RobotArmBase {
  /** Creates a new robotArm. */
  
  /*private double upperAngle;
  private double lowerAngle;*/
  private final static double kPArm = 0.1; 
  private final static double kIArm = 0.01;
  private final static double kDArm = 0.3;
  private final static double errorLimit = 1.5;
  protected Timer armTimer;
  private static double error = 0;
  private static double errorSum = 0; 
  private static double lastError = 0;
  private static double setpoint = 0;
  private double encoderInit = armMotor.getEncoder().getPosition(); 
  private double encoderPosition = encoderInit;
  private double lastTimestamp = armTimer.get();
  private double newSpeed = 0;
  
  @Override
  protected void otherConfigs() {
    armTimer= new Timer();
    armTimer.start();
  }
@Override
  public void periodic() {
      encoderPosition += armMotor.getEncoder().getPosition()*360/80;
      error = setpoint - encoderPosition;
      double dt = (armTimer.get()-lastTimestamp);
      if (Math.abs(error) < errorLimit) {
        errorSum += error*dt;
      } 
      
      double errorRate = (error-lastError)/dt;
      newSpeed = kPArm*error + kIArm*errorSum + kDArm*errorRate;
      if (newSpeed > 0.3) {
          newSpeed = 0.3;
      }
      if (Math.abs(setpoint-encoderPosition) < 0.1 && armMotor.get() < 0) {
          armMotor.set(0);
      } else {
        armMotor.set(newSpeed);
      }
      
      lastError = error;
      lastTimestamp = armTimer.get();
      
  }
    
  @Override
  protected void raiseArm(double maxArmPower) {
    
    System.out.println("robot arm raise called");
    setpoint = 75;
    armMotor.set(newSpeed);
  }
  @Override
  protected void lowerArm(double maxArmPower)  {
  
    System.out.println("robot arm lower called");
    setpoint = -75;
    armMotor.set(-newSpeed); 
  }
  @Override
  public void moveArm(boolean ArmUp) {
    if (ArmUp) {
      raiseArm(.3);
      if(!armStatus){
        armTimer.reset();
        armMotor.getEncoder().setPosition(0);
      }
      armStatus = true;
    } else {
      lowerArm(-.1);
      if(armStatus){
        armTimer.reset();
        armMotor.getEncoder().setPosition(0);
      }
      armStatus = false;
    }
    
  }
  @Override
  public void moveArmFirst() {
    lowerArm(-.3);
    if(armStatus){
      armTimer.reset();
    }
    armStatus = false;
    System.out.println("auto lower called");
  }
  
  
}
