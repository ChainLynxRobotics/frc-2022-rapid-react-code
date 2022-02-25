package frc.robot.subsystems;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DatumGyro extends SubsystemBase implements Gyro {

  private double gyroAngleZ;
  private DatumIMU datumIMU;
  private DatumIMU.DataPacket gyro;
  private double velocityInitial;


 static Stopwatch timer = new Stopwatch.StartNew(); 
 static double currentTimeStamp;
 static double lastTimeStamp;
  
  /** Creates a new DatumGyro. */
  public DatumGyro(String port) {
    datumIMU = new DatumIMU(port);
    gyro = datumIMU.getGyro();
    gyroAngleZ = 0;
    velocityInitial = 0;
    
  }


  public void periodic() {
      currentTimeStamp = timer.ElapsedMilliseconds;
      gyroAngleZ = gyroAngleZ + (((currentTimeStamp-lastTimeStamp)/1000)*velocityInitial;
      velocityInitial = velocityInitial +( gyro.z*(currentTimeStamp-lastTimeStamp));
      lastTimeStamp = currentTimeStamp;
  }

} 
