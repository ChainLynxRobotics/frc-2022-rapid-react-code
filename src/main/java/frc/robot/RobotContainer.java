package frc.robot;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveStyle;
import frc.robot.Constants.JoystickScaling;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map.*;
import edu.wpi.first.networktables.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private static RobotArmBase robotArm;
  private static DriveTrain driveTrain;
  private static OI m_OI;
  private static BallHandler ballHandler;
  private static boolean robotReversed;
  private static SendableChooser<Command> driveTrainChooser;
  
  private NetworkTableEntry driveSFA;
  private NetworkTableEntry driveSFB;
  private NetworkTableEntry driveSFC;
  private NetworkTableEntry driveSFD;
  private NetworkTableEntry driveSA;
  private NetworkTableEntry driveSB;

  //The container for the robot. Contains subsystems, OI devices, and commands. 
  private PowerDistribution powerDistribution;
  public RobotContainer() {
    driveTrain = new DriveTrain();
    m_OI = new OI();
    ballHandler = new BallHandler();
    //robotArm = new RobotArm();
    powerDistribution = new PowerDistribution();
    powerDistribution.clearStickyFaults();
    makeShuffleboardDriveConstants();
    chooseDriveStyle();
    startCommands();
    //configureCameras();
    }
  private void chooseDriveStyle(){
    driveTrainChooser= new SendableChooser<>();
    driveTrainChooser.setDefaultOption("arcadeDrive", new RunCommand(() -> driveTrain.drive(m_OI.getDriveStickRawAxis(1),m_OI.getDriverButton(2)?1:m_OI.getDriveStickRawAxis(0),getDriveMultiplier(),JoystickScaling.SQUARED_EXPONENTIAL,4,DriveStyle.NORMAL_ARCADE),driveTrain));
    driveTrainChooser.addOption("customTankDrive", new RunCommand(() -> driveTrain.drive(m_OI.getDriveStickRawAxis(1),m_OI.getDriverButton(2)?1:m_OI.getDriveStickRawAxis(0),getDriveMultiplier(),JoystickScaling.SQUARED_EXPONENTIAL,4,DriveStyle.CUSTOM_TANK), driveTrain));
    driveTrainChooser.addOption("arcadeTankDrive", new RunCommand(() -> driveTrain.drive(m_OI.getDriveStickRawAxis(1),m_OI.getDriverButton(2)?1:m_OI.getDriveStickRawAxis(0),getDriveMultiplier(),JoystickScaling.SQUARED_EXPONENTIAL,4,DriveStyle.ARCADE_TANK), driveTrain));
    driveTrainChooser.addOption("saturatedArcade", new RunCommand(() -> driveTrain.drive(m_OI.getDriveStickRawAxis(1),m_OI.getDriverButton(2)?1:m_OI.getDriveStickRawAxis(0),getDriveMultiplier(),JoystickScaling.SQUARED_EXPONENTIAL,4,DriveStyle.SATURATED_ARCADE), driveTrain));
    //sendable chooser option for arcade tank drive 
    SmartDashboard.putData(driveTrainChooser);
  }
  // this is where we will set up camera code
  /*private void configureCameras() {
    CameraServer.startAutomaticCapture("camera1",0);
    CameraServer.startAutomaticCapture("camera2",1);
  }
*/
  // this is the method where we are going to start all our commands to reduce clutter in RobotContainer method
   private void startCommands() {
    driveTrain.setDefaultCommand(driveTrainChooser.getSelected());
    ballHandler.setDefaultCommand(new RunCommand(() -> ballHandler.ballHandlerRunning(m_OI.getOperatorStickAxis(m_OI.getOperatorStickSliderAxis()),m_OI.getOperatorButton(1),m_OI.getDriverButton(3)),ballHandler));
    //robotArm.setDefaultCommand(new RunCommand(() -> robotArm.moveArm(m_OI.getOperatorButtons67Toggle()), robotArm));
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
    
    SmartDashboard.putBoolean("status/robottReversed", robotReversed);
    SmartDashboard.putNumber("status/speedmultiplier", driveMultiplier);
    SmartDashboard.putNumber("status/speedpercentageoutput", m_OI.getDriverButton(2)?1*driveMultiplier:m_OI.getDriveStickRawAxis(0)*driveMultiplier); // i am sorry this was genuinely the easiest solution i could come up with
    return driveMultiplier;
  }
  public void updateShuffleboard(){
    //SmartDashboard.putData(robotArm);
    updateConstants();

  }
  public DriveTrain getRobotDrive(){
    return driveTrain;
  }
  
  public Command getAutonomousDriveCommand() {
    // this code should work but i am very skeptical of stuff like this so it might not
    return new SequentialCommandGroup(new RunCommand(() -> ballHandler.ballHandlerRunning(.5,true,false),ballHandler).withTimeout(2),
    new RunCommand(() -> driveTrain.testDrive(-0.4, -0.4), driveTrain).withTimeout(3));
  }
  
  public void makeShuffleboardDriveConstants() {
    ShuffleboardTab driveConstants = Shuffleboard.getTab("drive constants");
    driveSFA = driveConstants.add("arcade drive scale factor A", ArcadeTankDrive.getSFA()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    driveSFB = driveConstants.add("arcade drive scale factor B", ArcadeTankDrive.getSFB()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    driveSFC = driveConstants.add("arcade drive scale factor C", ArcadeTankDrive.getSFC()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    driveSFD = driveConstants.add("arcade drive scale factor D", ArcadeTankDrive.getSFD()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    driveSA = driveConstants.add("arcade drive scale A", ArcadeTankDrive.getSA()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    driveSB = driveConstants.add("arcade drive scale B", ArcadeTankDrive.getSB()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
  }
  
  public void updateConstants() {
    ArcadeTankDrive.setSFA(driveSFA.getDouble(0.0));
    ArcadeTankDrive.setSFB(driveSFB.getDouble(0.0));
    ArcadeTankDrive.setSFC(driveSFC.getDouble(0.0));
    ArcadeTankDrive.setSFD(driveSFD.getDouble(0.0));
    ArcadeTankDrive.setSA(driveSA.getDouble(0.0));
    ArcadeTankDrive.setSB(driveSB.getDouble(0.0));
  }
  
}
