package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.RobotMap;

public class RobotArm extends RobotArmBase {

  private double offsetAngle; //in degrees
  private final static double kPArm = 0.1;
  private final static double kIArm = 0.01;
  private final static double kDArm = 0.5;
  private final static double errorLimit = 1.5;
  
 
  private static double error = 0;
  private static double errorSum = 0;
  private static double lastError = 0;
  private static double lastTimestamp = 0;
  private static double setpoint = 0; 
  private static double encoderInit = 0; 
  private static double lastTimestamp = Timer.getFPGATimestamp();
  private static double encoderInit = armMotor.getEncoder().getPosition()*360;
 
  
  @Override
  public void periodic() {
      System.out.println("arm encoder position" + armMotor.getEncoder().getPosition());
      //returns units in #rotations, which is converted to degrees
      double encoderPosition = encoderInit + armMotor.getEncoder().getPosition()*360;

      if (armMotor.get() > 0) {
          setpoint = 90;
      } else if (armMotor.get() < 0) {
          setpoint = 0;
      }

      //as arm approaches setpoint, it will slow down
      error = setpoint - encoderPosition;
      double dt = (Timer.getFPGATimestamp() - lastTimestamp);

      if (Math.abs(error) < errorLimit) {
        errorSum += error*dt;
      }


      double errorRate = (error - lastError)/dt;
      double newSpeed = kPArm*error + kIArm*errorSum + kDArm*errorRate;

      if(Math.abs(setpoint-encoderPosition) < 0.1 && armMotor.get() < 0) {
        armMotor.set() = 0;
      }

      armMotor.set(newSpeed);
      lastError = error;
      lastTimestamp = Timer.getFPGATimestamp();
  }

  public void raiseArm() {
    armMotor.set(newSpeed);
  }

  public void lowerArm()  {
    armMotor.set(-newSpeed);

    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      System.out.println("arm wait didn't work");
    }
    while(armMotor.getEncoder().getVelocity() < 0){
      //System.out.println("arm lowering");
    }
    armMotor.set(0);
    
  }
  
}
