// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry; // i am very upset about the amount of effort it took me to get this single stupid import to work i hate my life life is suffering and pain
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.SimulationConstants;


import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;


public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private CANSparkMax m_leftDriveFront, m_leftDriveBack;
  private CANSparkMax m_rightDriveFront, m_rightDriveBack;
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  private Boolean breakStatus;
  private DifferentialDrive m_drive;
  private Field2d fieldSim;
  private final DifferentialDriveOdometry m_odometry;
  private double cmPerTick;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  
  public DriveTrain() {
    
    m_leftDriveFront = new CANSparkMax(RobotMap.MOTOR_LEFT_MASTER_ID, MotorType.kBrushless);
    m_leftDriveBack= new CANSparkMax(RobotMap.MOTOR_LEFT_SLAVE_ID, MotorType.kBrushless);
    m_rightDriveFront= new CANSparkMax(RobotMap.MOTOR_RIGHT_MASTER_ID, MotorType.kBrushless);
    m_rightDriveBack = new CANSparkMax(RobotMap.MOTOR_RIGHT_SLAVE_ID, MotorType.kBrushless); 
    leftEncoder= m_leftDriveFront.getEncoder();
    rightEncoder= m_rightDriveFront.getEncoder();
    leftMotors = new MotorControllerGroup(m_leftDriveFront, m_leftDriveBack);
    rightMotors = new MotorControllerGroup(m_rightDriveFront, m_rightDriveBack);
    m_drive = new DifferentialDrive(leftMotors, rightMotors);
    breakStatus = true;
   
    
    resetEncoders();
    fieldSim = new Field2d();
    SmartDashboard.putData("status/fieldlocation", fieldSim);
    //we might want to zero heading here if we can get that to work
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
            DriveConstants.WHEEL_RADIUS,
            DriveConstants.TRACK_WIDTH,
            SimulationConstants.MEASUREMENT_NOISE);
      
      
      fieldSim = new Field2d();
      SmartDashboard.putData("Field", fieldSim);
          // to edit the values of this part of code edit the constants is Constants.java
      }
  }

  
  public void setBreakStatus(boolean breakOn){
    if (breakOn != breakStatus && breakOn == true){
      m_leftDriveFront.setIdleMode(IdleMode.kBrake);
      m_leftDriveBack.setIdleMode(IdleMode.kBrake);
      m_rightDriveFront.setIdleMode(IdleMode.kBrake);
      m_rightDriveBack.setIdleMode(IdleMode.kBrake);
    }
    else if(breakOn != breakStatus && breakOn == false){
      m_leftDriveFront.setIdleMode(IdleMode.kCoast);
      m_leftDriveBack.setIdleMode(IdleMode.kCoast);
      m_rightDriveFront.setIdleMode(IdleMode.kCoast);
      m_rightDriveBack.setIdleMode(IdleMode.kCoast);
    }
  }

      @Override
      public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the simulation,
        // and write the simulated positions and velocities to our simulated encoder and gyro. note we do not have these
        // We negate the right side so that positive voltages make the right side
        // move forward.
        m_drivetrainSimulator.setInputs(
            -leftMotors.get() * RobotController.getBatteryVoltage(),
            rightMotors.get() * RobotController.getBatteryVoltage());
        m_drivetrainSimulator.update(0.020);
        
        //m_gyroSim.setGyroAngleZ(-m_drivetrainSimulator.getHeading().getDegrees());
        //  note, the code that these are using may involve a drive function that controls the speed of the motors using volts directly
        // also only important for simulation so only look at this if you are having issues with simulation results
      }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if i remember correctly nothing has to be in here but i think it has something to do with the simulation so don't touch it for now
    if (RobotBase.isSimulation()) {  m_odometry.update(
          Rotation2d.fromDegrees(getHeading()),
          leftEncoder.getPosition(),
          rightEncoder.getPosition());
      fieldSim.setRobotPose(getPose());
      
    }
    getHeading();
    SmartDashboard.putNumber("status/robotspeedinmeterspersecond", DriveConstants.WHEEL_CIRCUMFERENCE/(m_leftDriveFront.getEncoder().getCountsPerRevolution() *4)); // displays speed in meters per second
    SmartDashboard.putData("status/drivetraindata", m_drive);
  }
  
  /*
  //unnessary with current code as resetting encoder resets heading
  public void zeroHeading() {
    gyro.reset();
  }
  */

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }
  
  // our drive command, this is what is called in robot.java
  // if this is not a command, it might be necessary to move this to a different command class where it is called by driveTrain
  public void drive(double turn, double throttle, boolean turnMode) {
    m_drive.curvatureDrive(-turn*Math.abs(turn),throttle*Math.abs(throttle), turnMode);
    
  }

  // this is necessary for a class in robot.java for the simulation
  // this is not important in any way outside of the simulation
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();

  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  // switch to a gyro if we cant get it working for encoders by the time we have a gyro
  public double getHeading() {
    cmPerTick =  DriveConstants.WHEEL_CIRCUMFERENCE * 100 / (leftEncoder.getCountsPerRevolution() *4); // cpr does not count for 4X scaling with this library
    double degreesPerTick = cmPerTick / (DriveConstants.WHEEL_RADIUS * 100) * (180 * Math.PI);
    // multiplied by 100 to get in CM
    double encoderDifference = leftEncoder.getPosition() - rightEncoder.getPosition();
    double turningValue = encoderDifference * degreesPerTick;
    
    double finalDegrees= (turningValue % 360) * -1;
    //System.out.println("the heading of the robot is" + finalDegrees);
    SmartDashboard.putNumber("status/robotheading", finalDegrees);
    return finalDegrees;
    
  }
  // this might throw an error, it wont be used for week one though so figure it out later if it does
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return  new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity()*cmPerTick/100,rightEncoder.getVelocity()*cmPerTick/100);
  }
  //tank drive volts method solely for following trajectories
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
  // this is for testing but more generic than other test code so probobly worth keeping in after the others are gone
  public void testDrive(double leftSpeed, double rightSpeed){
    m_drive.tankDrive(leftSpeed, -rightSpeed);
  }
}
