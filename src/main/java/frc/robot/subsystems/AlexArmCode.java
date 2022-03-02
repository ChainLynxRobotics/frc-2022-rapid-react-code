package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.abstractSubsystems.RobotArmBase;

public class AlexArmCode extends RobotArmBase {
    private double avgDeltaVelocity;
    private double prevVelocity;
    @Override
    protected void otherConfigs() {
        
        avgDeltaVelocity = 0;
        prevVelocity = 0; 
    }

    @Override
    protected void raiseArm() {
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
        
    }

    @Override
    protected void lowerArm() {
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
        
    }

    @Override
    public void moveArm(boolean ArmUp) {
        // code for this Alex,dont try to make the code use other methods
    }
    
}
