package frc.robot.subsystems;
import com.revrobotics.CANPIDController;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first;
import frc.robot.constants;
import frc.robot.shootingHashMaps;
import edu.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.PWMVictorSPX;


public class shooter extends CommandBase implements PIDSubsystem{
    //new motors used for shooting mechanism
    private VictorSPX leftMotor1 = new VictorSPX(0);
    private VictorSPX leftMotor2 = new VictorSPX(1);
    private VictorSPX rightMotor1 = new VictorSPX(2);
    private VictorSPX rightMotor2 = new VictorSPX(3);

    private leftMotorControllerGroup = new leftMotorControllerGroup(leftMotor1, leftMotor2);
    private rightMotorControllerGroup = new rightMotorControllerGroup(rightMotor1, rightMotor2);
    
    //use a hood to angle balls (upper and lower hub)
    //placeholder gear ratio times degrees per count per revolution
    public static final double shooterAngleScale = ((1/120)*(360/42));
    // hood and flywheel initializations and PID
    private final WPI_CANEncoder shooterEncoder; 
    private final WPI_VictorSPX shooterAngleMotor;
    private final WPI_CANEncoder angleEncoder; 

    private final CANPIDController hoodPID;
    private static final double kPHood = 0.05;
    private static final double kIHood = 0;
    private static final double kDHood = 0;

    
    private final CANPIDController flywheelPID;
    private static final double kPFlywheel = 0.05;
    private static final double kIFlywheel = 0;
    private static final double kDFlywheel = 0;


    private double targetSpeed;
    private double targetHoodAngle;


    public static final double maximum height = velocity^2/19.6; 


    public Shooter() {
        
        public double getAngle(){
            double position = encoders.shooterAngleEncoder.getPosition();
            return position; 
        }
        

        public void Ultrasonic(int pingChannel, int echoChannel) {
            Ultrasonic distSensor = new Ultrasonic(1, 2); //creates an instance of distance sensor
            Ultrasonic.setAutomaticMode(true);
            double range = distSensor.GetRangeInches();
        }
        
        
        public sensor(int pingChannel, int echoChannel) { 
            if(distSensor.GetRangeInches() < 20) {
            drive.autonomousPeriodic(0, 0);
            }
            return range; //use this distance from hub to calculate shooting angle
        }

        public init() {
            errorSum = 0;
            lastError = 0;
            lastTimestamp = Timer.getFPGATimestamp; 
        }

        shooterEncoder = new WPI_CANVictorSPX(Constants.RobotMap, MotorType.kBrushless)
        intialVictorSPX(shooterEncoder);
        shooterEncoder.setInverted(Constants.kShooterInverted);
        shooterEncoder.setPositionConversionFactor(ShooterAngleScale);

        shooterAngleEncoder = shooterAngleMotor.getEncoder();
        flywheelPID = new CANPIDController(shooterEncoder);

        flywheelPID.set(kPFlywheel);
        flywheelPID.set(kIFlywheel);
        flywheelPID.set(kDFlywheel);


        shooterAngleMotor = new WPI_CANVictorSPX(Constants.kShooterAngle, MotorType.kBrushless);
        initVictorSPX(shooterAngleMotor);
        shooterAngleMotor.setInverted(Constants.kShooterAngleInverted);
        shooterAngleMotor.setIdleMode(WPI_CANVictorSPX.IdleMode.kBrake);
        shooterAngleMotor.setSoftLimit(WPI_CANVictorSPX.SoftLimitDirection.kForward, kHoodReverseSoftLimit);
        shooterAngleMotor.setSoftLimit(WPI_CANVictorSPX.SoftLimitDirection.kReverse, kHoodReverseSoftLimit);

        hoodPID = new CANPIDController(shooterAngleMotor);
        hoodPID.setHoodAngleSpeed();

        hoodPID.set(kPHood);
        hoodPID.set(kIHood);
        hoodPID.set(kDHood);

    }
    
    private void initVictorSPX(WPI_VictorSPX VictorSPX) {
        VictorSPX.setCurrrentLimit(60);
    }

    public void enableHoodLimits(boolean enabled) {
        shooterAngleMotor.enableSoftLimit(WPI_CANVictorSPX.SoftLimitDirection.kForward, enabled);
        shooterAngleMotor.enableSoftLimit(WPI_CANVictorSPX.SoftLimitDirection.kReverse, enabled);
    }


    //velocity modulation
    public void shoot(double percentOutput) {
        shooterAngleMotor.set(percentOutput);
    }


    //goal angle
    public void setHoodAngleDegrees(double targetHoodAngle) {
        this.targetHoodAngle = targetHoodAngle;
        hoodPID.setReference(targetHoodAngle, position);
    }

    public void setHoodAngleSpeed(double percentOutput) {
        shooterAngleMotor.set(percentOutput);
    }

    boolean isHoodAtTheshold() {
       return (getHoodCurrent() > currentThreshold());
    }

    //set closed loop (feedback acceptance) here 
    
    public double getSpeed(){
        double speed = shootingHashMaps.convert(range+1.59);
        return speed;
    }
    
     public double getSpeedError(){
        double currentSpeed = getSpeed();
        double error = targetSpeed - currentSpeed;
        return error;
      }
    
      public double getHoodAngleError() {
        return targetHoodAngle - getAngle();
      }
    
      public void hoodEncoder() {
          shooterAngleEncoder.setPosition(0);
      }

      public void timestamp() {
        dt = Timer.getFPGATimestamp() - lastTimestamp;
      }

     //for velocity (hood)
        if (error < 1) {
            errorSum += error; 
      }

      double errorRate = (error - lastError)/dt;
      double outputSpeed = kPHood*error + kIHood*errorSum + kDHood*errorRate;

      leftMotorControllerGroup.set(outputSpeed);
      rightMotorControllerGroup.set(-outputSpeed);
    

      public void stop() {
        shooterAngleMotor.stopMotor();
      }


      @Override
      public void Periodic() {
        double currentMotor = shooterAngleMotor.getOutputCurrent();
      }

}
