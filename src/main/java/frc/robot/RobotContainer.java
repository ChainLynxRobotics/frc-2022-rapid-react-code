// i chose to comment out this class because i had an error trying to run a command
// we will probobly use this during the competion season to help signifigantly with organizing our subsystems and code in general
// but for now at least for demonstation purposes we will not use this class

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DriveTrain;

import frc.robot.subsystems.RobotArm;
import frc.robot.subsystems.RobotArmPID;
import frc.robot.subsystems.abstractSubsystems.RobotArmBase;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static RobotArmBase robotArm;
  private static DriveTrain driveTrain;
  private static OI m_OI;
  private static BallHandler ballHandler;
  private static boolean robotReversed;
  //The container for the robot. Contains subsystems, OI devices, and commands. 
  private PowerDistribution powerDistribution;
  public RobotContainer() {
    driveTrain = new DriveTrain();
    m_OI = new OI();
    ballHandler = new BallHandler();
    robotArm = new RobotArmPID();
    powerDistribution = new PowerDistribution();
    powerDistribution.clearStickyFaults();
    startCommands();
    configureCameras();
    }
  // this is where we will set up camera code
  private void configureCameras() {
    CameraServer.startAutomaticCapture("camera1",0);
    CameraServer.startAutomaticCapture("camera2",1);
  }

  // this is the method where we are going to start all our commands to reduce clutter in RobotContainer method
   private void startCommands() {
    // driveTrain.setDefaultCommand(new RunCommand(() -> driveTrain.drive(m_OI.getDriveStickRawAxis(1)*getDriveMultiplier(),m_OI.getDriverButton(2)?1*getDriveMultiplier():m_OI.getDriveStickRawAxis(0)*getDriveMultiplier(),m_OI.getDriverButton(3)),driveTrain));
    // ballHandler.setDefaultCommand(new RunCommand(() -> ballHandler.ballHandlerRunning(m_OI.getOperatorStickAxis(m_OI.getOperatorStickSliderAxis()),m_OI.getOperatorButton(1),m_OI.getDriverButton(3)),ballHandler));
    robotArm.setDefaultCommand(new RunCommand(() -> robotArm.moveArm(m_OI.getOperatorButtons67Toggle()), robotArm));
   }
   // method to allow for constant multiplier for drivetrain speed
   private double getDriveMultiplier(){
    // this makes the z axis slider go from 0->1 instead of -1->1
    double driveMultiplier = ((m_OI.getDriveStickRawAxis(m_OI.getDriveStickSliderAxis()) + 1) / 2);
    // this codes to have the robot break when the scaler sets the speed to 0
    driveMultiplier = m_OI.getDriverButton(14)?0:driveMultiplier;
    
    driveTrain.setBreakStatus(driveMultiplier == 0);
    
    driveMultiplier =  m_OI.getDriverButton(1)?-1:driveMultiplier;
    robotReversed= m_OI.getDriverButton(1);// this is ugly and bad code: it works
    driveMultiplier *= .84;
    
    SmartDashboard.putBoolean("status/robottReversed", robotReversed);
    SmartDashboard.putNumber("status/speedmultiplier", driveMultiplier);
    SmartDashboard.putNumber("status/speedpercentageoutput", m_OI.getDriverButton(2)?1*driveMultiplier:m_OI.getDriveStickRawAxis(0)*driveMultiplier); // i am sorry this was genuinely the easiest solution i could come up with
    return driveMultiplier;
  }
  public void updateShuffleboard(){
    SmartDashboard.putData(robotArm);
  }
  
  public void updateShuffleboard(){
    SmartDashboard.putData(robotArm);
  }
  
  public DriveTrain getRobotDrive(){
    return driveTrain;
  }
  
  public Command getAutonomousDriveCommand() {
    // this code should work but i am very skeptical of stuff like this so it might not
    return new SequentialCommandGroup(new RunCommand(() -> ballHandler.ballHandlerRunning(.5,true,false),ballHandler).withTimeout(2),
    new RunCommand(() -> driveTrain.testDrive(-0.4, -0.4), driveTrain).withTimeout(3),new RunCommand(() -> robotArm.moveArm(true), robotArm));
  }
  
   public void makeShuffleboardDriveConstants() {
    driveConstants = Shuffleboard.getTab("drive constants");
    kP = driveConstants.add("kPArm", robotArm.getkP(kPArm)).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    kI = driveConstants.add("kIArm", robotArm.getkI(kIArm)).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    kD = driveConstants.add("kDArm", robotArm.getkD(kDArm)).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    double constantUser1 = kP.getDouble(1.0);
    double constantUser2 = kI.getDouble(1.0);
    double constantUser3 = kD.getDouble(1.0);
  }
  
}

