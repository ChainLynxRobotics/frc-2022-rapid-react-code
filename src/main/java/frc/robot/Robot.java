

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import main.java.frc.robot.tests.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
// when i was coding this class, i couldn't get robot container to work right so i decided to just not use it at least for this model
// big note, this code can support simulation BUT it lacks odometry and i haven't accounted for encoders yet, because they are not on our robot
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer robotContainer;
  // the subsystems are declared here, and not in robotInit as you would assume to workaround the 
  //fact that simulation periodic actucally runs BEFORE robotInit 
  //so i had to declare them here to avoid a null pointer error
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  @Override
  public void robotInit() {
    // i did not code for the smart dashboard and also didn't code any autonomous code so i only have the drive command in here
    //in the future this should be moved to teleop init, likely in a command form
    robotContainer = new RobotContainer();
    
    setNetworkTablesFlushEnabled(true);
  }
  @Override
  public void simulationPeriodic() {
    // this is where you put code that needs to be periodically run for the simulation
    //IMPORTANT NOTE: this code runs at least once before robotInit when simulating the code so make sure that if any methods need subsystems, they are called before robotInit
    double drawCurrent = robotContainer.getRobotDrive().getDrawnCurrentAmps();
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
    RoboRioSim.setVInVoltage(loadedVoltage);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run(); // this method is what causes commands to continue to run 
    //if there is any issue in the code currently, its that driveTrain.drive likely isn't a command
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {CommandScheduler.getInstance().cancelAll();
  robotContainer.getRobotDrive().setBreakStatus(true);
  }

  @Override
  public void disabledPeriodic() {}
 
  @Override
  public void disabledExit() {
      robotContainer.getRobotDrive().setBreakStatus(false);
  }
  


  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override // when we add autonomous code we should initialize it here
  public void autonomousInit() { 
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // this should actually be empty unless we move the command schedueler out of robot periodic and into this and teleop periodic
    // the reason we leave this part of the code empty, is the commands should handle the robots movement on their own, we just initialze that for autonomous
  } 

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // here we would also initialize the teleop command but as i said that doesn't make sense with the current code makeup
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // same as autonomous periodic
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    
    gyro_initialize();
    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
    gyro_update();
    
  }
  
}
