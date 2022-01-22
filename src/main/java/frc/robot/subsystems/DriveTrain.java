// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry; // i am very upset about the amount of effort it took me to get this single stupid import to work i hate my life life is suffering and pain
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.SimulationConstants;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;


public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private WPI_VictorSPX m_leftDriveFront, m_leftDriveBack;
  private WPI_VictorSPX m_rightDriveFront, m_rightDriveBack;
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;
  private final ADXRS450_Gyro m_gyro;

  
  
  private DifferentialDrive m_drive;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private Field2d fieldSim;
  private ADXRS450_GyroSim m_gyroSim;
  private final DifferentialDriveOdometry m_odometry;

  
  public DriveTrain() {
    
    m_leftDriveFront = new WPI_VictorSPX(RobotMap.MOTOR_LEFT_MASTER_ID );
    m_leftDriveBack= new WPI_VictorSPX(RobotMap.MOTOR_LEFT_SLAVE_ID );
    m_rightDriveFront= new WPI_VictorSPX(RobotMap.MOTOR_RIGHT_MASTER_ID );
    m_rightDriveBack = new WPI_VictorSPX(RobotMap.MOTOR_RIGHT_SLAVE_ID );
    
    leftMotors = new MotorControllerGroup(m_leftDriveFront, m_leftDriveBack);
    rightMotors = new MotorControllerGroup(m_rightDriveFront, m_rightDriveBack);
    rightMotors.setInverted(RobotMap.RIGHT_SIDE_INVERTED);
    leftMotors.setInverted(RobotMap.LEFT_SIDE_INVERTED);
    m_drive = new DifferentialDrive(leftMotors, rightMotors);
    
    m_leftEncoder = 
    new Encoder(RobotMap.MOTOR_LEFT_MASTER_ID,RobotMap.MOTOR_LEFT_SLAVE_ID,RobotMap.LEFT_SIDE_INVERTED);
    m_rightEncoder = 
    new Encoder(RobotMap.MOTOR_RIGHT_MASTER_ID,RobotMap.MOTOR_RIGHT_SLAVE_ID,RobotMap.RIGHT_SIDE_INVERTED);
    m_leftEncoder.setDistancePerPulse(DriveConstants.ENCODER_PULSE_DISTANCE);
    m_rightEncoder.setDistancePerPulse(DriveConstants.ENCODER_PULSE_DISTANCE);
    m_gyro = new ADXRS450_Gyro();
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    // this code basically assigns motor controllers to variables, then groups for the sides, then the drivetrain and assigns values to our encoders
    // also inverts the right side to make sure the motor moves straight
    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
            SimulationConstants.MOTOR_QUANTITY,
            SimulationConstants.GEAR_RATIO,
            SimulationConstants.MOMENT_OF_INERTIA,
            SimulationConstants.DRIVETRAIN_WEIGHT,
            SimulationConstants.WHEEL_RADIUS,
            DriveConstants.TRACK_WIDTH,
            SimulationConstants.MEASUREMENT_NOISE);
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      m_gyroSim = new ADXRS450_GyroSim(m_gyro);
      fieldSim = new Field2d();
      SmartDashboard.putData("Field", fieldSim);
          // to edit the values of this part of code edit the constants is Constants.java
      }
  }

  


      @Override
      public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the simulation,
        // and write the simulated positions and velocities to our simulated encoder and gyro. note we do not have these
        // We negate the right side so that positive voltages make the right side
        // move forward.
        m_drivetrainSimulator.setInputs(
            leftMotors.get() * RobotController.getBatteryVoltage(),
            -rightMotors.get() * RobotController.getBatteryVoltage());
        m_drivetrainSimulator.update(0.020);
        m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
        //  note, the code that these are using may involve a drive function that controls the speed of the motors using volts directly
        // also only important for simulation so only look at this if you are having issues with simulation results
      }
     

      
      


    


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if i remember correctly nothing has to be in here but i think it has something to do with the simulation so don't touch it for now
    if (RobotBase.isSimulation()) {  m_odometry.update(
          Rotation2d.fromDegrees(getHeading()),
          m_leftEncoder.getDistance(),
          m_rightEncoder.getDistance());
      fieldSim.setRobotPose(getPose());
      
    }
  }
  
  
  public void zeroHeading() {
    m_gyro.reset();
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }
  
  // our drive command, this is what is called in robot.java
  // if this is not a command, it might be necessary to move this to a different command class where it is called by driveTrain
  public void drive(double turn, double throttle) {
    m_drive.arcadeDrive(-turn,throttle, true);
    
  }
  // this is necessary for a class in robot.java for the simulation
  // this is not important in any way outside of the simulation
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();

  }
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (SimulationConstants.SIM_GYRO_INVERTED ? -1.0 : 1.0);
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return  new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(leftVolts),Math.abs(rightVolts))> batteryVoltage ){
      leftVolts *= batteryVoltage /12.0;
      rightVolts *= batteryVoltage/ 12.0;

    }
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    m_drive.feed();

  }
}
