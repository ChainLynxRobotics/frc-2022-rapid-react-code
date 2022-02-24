// i chose to comment out this class because i had an error trying to run a command
// we will probobly use this during the competion season to help signifigantly with organizing our subsystems and code in general
// but for now at least for demonstation purposes we will not use this class

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint; //pathweaver has this so we dont need to use this
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DriveTrain;

import frc.robot.subsystems.RobotArm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static RobotArm robotArm;
  private static DriveTrain driveTrain;
  private static OI m_OI;
  private static BallHandler ballHandler;
  private static boolean robotReversed;
  //The container for the robot. Contains subsystems, OI devices, and commands. 
  
  public RobotContainer() {
    driveTrain = new DriveTrain();
    m_OI = new OI();
    ballHandler = new BallHandler();
    robotArm = new RobotArm();
    
    // Configure the button bindings
    configureButtons();
    // start commands
    startCommands();
    
  
    }
  private void configureButtons() {
    // configure the arm's toggle
    // you can add a boolean to turn the command off if you want it to be overridden and turned off (ie for auto)
    m_OI.getOperatorButton2().toggleWhenActive(new StartEndCommand(robotArm::raiseArm, robotArm::lowerArm, robotArm));
    

  }
  // this is the method where we are going to start all our commands to reduce clutter in RobotContainer method
   private void startCommands() {
    driveTrain.setDefaultCommand(new RunCommand(() -> driveTrain.drive(m_OI.getDriveStickRawAxis(1)*getDriveMultiplier(),m_OI.getDriverButton(2)?1*getDriveMultiplier():m_OI.getDriveStickRawAxis(0)*getDriveMultiplier() ),driveTrain));
    ballHandler.setDefaultCommand(new RunCommand(() -> ballHandler.ballHandlerRunning(m_OI.getOperatorStickAxis(1),m_OI.getOperatorButton(1)),ballHandler));
    
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
    System.out.println("the multiplier of the robots speed is" + driveMultiplier);
    SmartDashboard.putBoolean("status/robottReversed", robotReversed);
    SmartDashboard.putNumber("status/speedmultiplier", driveMultiplier);
    SmartDashboard.putNumber("status/speedpercentageoutput", m_OI.getDriverButton(2)?1*driveMultiplier:m_OI.getDriveStickRawAxis(0)*driveMultiplier); // i am sorry this was genuinely the easiest solution i could come up with
    return driveMultiplier;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public DriveTrain getRobotDrive() {
    return driveTrain;
  }
  
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(SimulationConstants.VOLTS, SimulationConstants.VOLTS_SCNDS_PER_METER, SimulationConstants.VOLTS_SCNDS_SQUARED_PER_METER),
       DriveConstants.DRIVE_KINEMATICS, DriveConstants.AUTO_VOLTAGE_CONSTRAINT);
    */
    // VERY IMPORTANT MAKE SURE TO UPDATE THIS DIRECTORY WHEN YOU RUN THIS CODE TO MATCH YOUR OWN FOLDER OR THE CODE WILL NOT WORK
    String trajectoryJSON = "C:\\Users\\ChainLynx\\Documents\\frc-2022-rapid-react-code\\PathWeaver\\output\\3pointspath.wpilib.json";
    Trajectory pathWeaverTrajectory = new Trajectory();
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      pathWeaverTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch(IOException autoEX){
      DriverStation.reportError("unable to open trajectory" + trajectoryJSON , autoEX.getStackTrace());
    }
    
    driveTrain.resetOdometry(pathWeaverTrajectory.getInitialPose()); // sets robots internal location to its location set in the trajectory
    
    
    RamseteCommand autoCommand = new RamseteCommand(
      pathWeaverTrajectory,
      driveTrain::getPose, 
      new RamseteController(), 
      new SimpleMotorFeedforward(SimulationConstants.VOLTS, SimulationConstants.VOLTS_SCNDS_PER_METER, SimulationConstants.VOLTS_SCNDS_SQUARED_PER_METER),
      DriveConstants.DRIVE_KINEMATICS, 
      driveTrain::getWheelSpeeds, 
      new PIDController(DriveConstants.AUTO_DRIVE_SPEED, 0, 0),
      new PIDController(DriveConstants.AUTO_DRIVE_SPEED, 0, 0),
      driveTrain::tankDriveVolts,
      driveTrain);
    
    driveTrain.resetOdometry(pathWeaverTrajectory.getInitialPose());
    return autoCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
    
  }
  
}

