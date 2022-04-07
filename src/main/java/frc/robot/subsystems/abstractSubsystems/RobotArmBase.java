// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.abstractSubsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// extend this class if you want to make your own version of robot arm, this will make your life signifigantly easier
public abstract class RobotArmBase extends SubsystemBase {
  /** Creates a new RobotArmBase. */

  protected CANSparkMax armMotor;
  protected boolean armStatus;

  // you dont need to write a constructor, just another benifit
  public RobotArmBase() {
    armMotor = new CANSparkMax(RobotMap.ROBOT_ARM_MOTOR_ID, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSmartCurrentLimit(40);

    // change this to true after code is tested
    armStatus = true;
    otherConfigs();
  }

  // use this method to set initial values for variables of an arm subsystem
  protected abstract void otherConfigs();

  // here put code to run when the arm is getting lower/ is low
  protected abstract void lowerArm();

  // here put code to run when the arm is raising / is raised
  protected abstract void raiseArm();

  // this is the method that is called by other objects, DO NOT OVERRIDE
  public abstract void moveArm(boolean ArmUp);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("motor temperature" + armMotor.getMotorTemperature());
    SmartDashboard.putNumber("status/armmotortemperature", armMotor.getMotorTemperature());

  }
}
