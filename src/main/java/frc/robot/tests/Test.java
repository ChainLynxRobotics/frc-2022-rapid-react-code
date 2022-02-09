package main.java.frc.robot.tests;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Test {

    protected static ADIS16448_IMU m_gyro = new ADIS16448_IMU();;

    public void gyro_initialize() {

        //m_gyro.calibrate();

    }

    public void gyro_update() {

        SmartDashboard.putNumber("gyroscope_X", m_gyro.getGyroAngleX());
        SmartDashboard.putNumber("gyroscope_Y", m_gyro.getGyroAngleY());
        SmartDashboard.putNumber("gyroscope_Z", m_gyro.getGyroAngleZ());

    }

}
