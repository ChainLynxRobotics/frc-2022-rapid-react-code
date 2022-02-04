// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

public class RobotArm extends SubsystemBase {
  /** Creates a new robotArm. */
  private DoubleSolenoid pneumaticArm;
  private boolean lastInput;
  public RobotArm() {
    DoubleSolenoid pneumaticArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.ROBOT_ARM_SOLENOIDS[0], RobotMap.ROBOT_ARM_SOLENOIDS[1]);
    pneumaticArm.set(DoubleSolenoid.Value.kReverse);
  }
  public void toggleHeight(boolean inputStatus){
    if(inputStatus != lastInput && inputStatus == true ){
      pneumaticArm.toggle();
    }
    else if(inputStatus != lastInput){
      lastInput = inputStatus;
    }

  }

  
}
