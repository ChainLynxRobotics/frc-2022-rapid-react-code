package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.abstractSubsystems.RobotArmBase;


public class RobotArmPID extends RobotArmBase {
    protected Timer armTimer;
    public static final double kPArm = 0.1;
    public static final double kIArm = 0;
    public static final double kDArm = 0;
    public static final double errorLimit = 1.5;
    private static double error = 0;
    private static double errorSum = 0;
    private static double lastError = 0;
    private static double setpoint = 75; //down position
    private double encoderInit = 0;
    private double encoderPosition = encoderInit;
    private double lastTimestamp = armTimer.get();
    private double newSpeed = 0;
    private double dt = 0;
    private double errorRate = 0;
 
    @Override
  protected void otherConfigs() {
    armTimer= new Timer();
    armTimer.start();
  }
    
  @Override
  public void periodic() {
    encoderPosition += armMotor.getEncoder().getPosition()*360;
    error = setpoint - encoderPosition;
    dt = armTimer.get() - lastTimestamp;

    if(Math.abs(error) < errorLimit) {
        errorSum += error; 
    }

    errorRate = (error - lastError)/dt;
    newSpeed = kPArm*error+kIArm*errorSum+kDArm*errorRate;

    if (newSpeed > 0.3) {
        newSpeed = 0.3;
    }
    
    if(Math.abs(setpoint-encoderPosition) < 0.1 && armMotor.get() < 0) {
        newSpeed = 0;
    } else if(Math.abs(setpoint-encoderPosition) < 35 && armMotor.get() > 0) {
        newSpeed = 0.1;
    }

    lastError = error;
    lastTimestamp = armTimer.get();
  }


  @Override
  protected void raiseArm() {
    setpoint = 0;
    System.out.println("robot arm raise called");
    armMotor.set(newSpeed); //.3 
    
  }
  @Override
  protected void lowerArm()  {
    setpoint = 75;
    System.out.println("robot arm lower called");
    armMotor.set(-newSpeed); //-.3
    
    
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
