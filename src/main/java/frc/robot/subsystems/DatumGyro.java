// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DatumGyro extends SubsystemBase implements Gyro {

  private double gyroAngleZ;
  private DatumIMU datumIMU;
  private DatumIMU.DataPacket gyro;
  private double velocityInitial;
  
  /** Creates a new DatumGyro. */
  public DatumGyro(String port) {
    datumIMU = new DatumIMU(port);
    gyro = datumIMU.getGyro();
    gyroAngleZ = 0;
    velocityInitial = 0;
    
  }

  @Override
  public void periodic() {
    
    gyroAngleZ = gyroAngleZ + (velocityInitial * 20 + 0.5 * gyro.z * 20 * 20);
    velocityInitial = velocityInitial +( gyro.z *20);
    // This method will be called once per scheduler run

  }

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void calibrate() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public double getAngle() {
    
    return gyroAngleZ;
  }

  public double getAngle(char axis) throws Exception{

    switch (axis) {
      case 'x':
        
        break;
      case 'y':

        break;
      case 'z':

        break;
      default:

        throw new Exception("not a valid axis");

    }

    return 0;

  }

  @Override
  public double getRate() {
    // TODO Auto-generated method stub
    return 0;
  }
}

