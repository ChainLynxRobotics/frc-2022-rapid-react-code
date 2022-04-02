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
    
    armMotor.set(.3);
    System.out.println("arm raised");
  }
  @Override
  protected void lowerArm()  {
  
    armMotor.set(-.15);
    
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