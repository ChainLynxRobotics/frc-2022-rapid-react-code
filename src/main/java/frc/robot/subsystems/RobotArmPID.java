package frc.robot.subsystems;

import java.lang.Math;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.abstractSubsystems.RobotArmBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class RobotArmPID extends RobotArmBase {
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
  private double armSpeed = 0;
  private double maxArmPower;

  //private ShuffleboardTab PIDTab = ShuffleBoard.getTab("PIDArm");
  //private NetworkTableEntry angularSpeedEntry = PIDTab.add("SpeedVelocity", 0).getEntry(); //use in periodic
  
  @Override
  protected void otherConfigs() {
    armTimer = new Timer();
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

      //Kv*v*2pi*newSpeed/60 sec
      armSpeed = 473*12*2*Math.PI*newSpeed/60;

      //SmartDashboard.putNumber("angularSpeed", armSpeed);
      //angularSpeedEntry.setDouble(armSpeed);

      lastError = error;
      lastTimestamp = armTimer.get();
  }
    
  @Override
  protected void raiseArm() {
    
    System.out.println("robot arm raise called");
    setpoint = 75;
    armMotor.set(newSpeed);
  }

  @Override
  protected void lowerArm()  {
    System.out.println("robot arm lower called");
    setpoint = -75;
    armMotor.set(-newSpeed); 
  }

  @Override
  public void moveArm(boolean ArmUp) {
    if (ArmUp) {
      raiseArm();
      if(!armStatus){
        armTimer.reset();
        armMotor.getEncoder().setPosition(0);
      }
      armStatus = true;
    } else {
      lowerArm();
      if(armStatus){
        armTimer.reset();
        armMotor.getEncoder().setPosition(0);
      }
      armStatus = false;
    }
    
  }
  

  
  
}
