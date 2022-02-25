// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
    // i spent 2 hours before i realized i just needed to type new ffs me
    // start commands
    startCommands();
    
    }
  
  

  // this is the method where we are going to start all our commands to reduce clutter in RobotContainer method
   private void startCommands() {
    driveTrain.setDefaultCommand(new RunCommand(() -> driveTrain.drive(m_OI.getDriveStickRawAxis(1)*getDriveMultiplier(),m_OI.getDriverButton(2)?1*getDriveMultiplier():m_OI.getDriveStickRawAxis(0)*getDriveMultiplier() ),driveTrain));
    ballHandler.setDefaultCommand(new RunCommand(() -> ballHandler.ballHandlerRunning(m_OI.getOperatorStickAxis(1),m_OI.getOperatorButton(1)),ballHandler));
    robotArm.setDefaultCommand(new ConditionalCommand(new RunCommand(() -> robotArm.raiseArm(),robotArm),new RunCommand(() -> robotArm.lowerArm(), robotArm),() -> m_OI.getOperatorButton2Toggle()));
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
  
  public Command getAutonomousDriveCommand() {
    return new SequentialCommandGroup(new StartEndCommand(() -> ballHandler.ballHandlerRunning(1,false), () -> ballHandler.ballHandlerRunning(0,false),ballHandler).withTimeout(2),
    new RunCommand(() -> driveTrain.tankDriveVolts(-0.4, -0.4), driveTrain).withTimeout(4));
    
  }




  
}

