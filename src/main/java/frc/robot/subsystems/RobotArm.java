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
  protected void raiseArm(double operatorMultiplier) {
    
    armMotor.set(.3*operatorMultiplier);
    System.out.println("arm raised");
  }
  @Override
  protected void lowerArm(double operatorMultiplier)  {
  
    armMotor.set(-.15*operatorMultiplier);
    
  }

  @Override
  public void moveArm(boolean ArmUp, double operatorMultiplier){
    if (ArmUp) {
      raiseArm(operatorMultiplier);
      if(!armStatus){
        armTimer.reset();
      }
      armStatus = true;
    } else {
      lowerArm(operatorMultiplier);
      if(armStatus){
        armTimer.reset();
      }
      armStatus = false;
    }
    
  }


  
  
}