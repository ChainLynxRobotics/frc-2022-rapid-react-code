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
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private static DriveTrain driveTrain;
  private static OI m_OI;
  
  //The container for the robot. Contains subsystems, OI devices, and commands. 

  public RobotContainer() {
    driveTrain = new DriveTrain();
    m_OI = new OI();
    // Configure the button bindings
    configureButtonBindings();
    driveTrain.setDefaultCommand(new RunCommand(() -> driveTrain.drive(m_OI.getJoystick1RawAxis(1)*getDriveMultiplier(),m_OI.getJoystick1RawAxis(0)*getDriveMultiplier() ),driveTrain));
  
    }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private double getDriveMultiplier(){
    double driveMultiplier = ((-m_OI.getJoystick1RawAxis(3) + 1) / 2);
    // this codes to have the robot break when the scaler sets the speed to 0
    if(driveMultiplier == 0){
      driveTrain.setBreakStatus(true);
    }
    else{
      driveTrain.setBreakStatus(false);
    }
    return driveMultiplier;
  }
   private void configureButtonBindings() {}
  
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
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(SimulationConstants.VOLTS, SimulationConstants.VOLTS_SCNDS_PER_METER, SimulationConstants.VOLTS_SCNDS_SQUARED_PER_METER),
       DriveConstants.DRIVE_KINEMATICS, DriveConstants.AUTO_VOLTAGE_CONSTRAINT);
    
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

