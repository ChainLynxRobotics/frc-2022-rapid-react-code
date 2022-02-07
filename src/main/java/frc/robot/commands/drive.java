//this ended up being unessary so i just nuked the code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class drive extends CommandBase {
  
  DriveTrain m_driveTrain;
  double m_throttle;
  double m_turn;
  
  public drive(DriveTrain m_driveTrain, double m_throttle, double m_turn) {
    m_driveTrain= this.m_driveTrain;
    m_throttle= this.m_throttle;
    m_turn = this.m_turn;
    
    
    
    addRequirements(Robot.driveTrain); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {m_driveTrain.drive(m_throttle,m_turn);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
*/