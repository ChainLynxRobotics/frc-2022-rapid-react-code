package frc.robot.subsystems.abstractSubsystems;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveStyle;
import frc.robot.Constants.JoystickScaling;
import edu.wpi.first.wpilibj.Timer;
public class CustomTankDrive extends RobotDriveBase implements Sendable, AutoCloseable{
    private final MotorControllerGroup leftMotors;
    private final MotorControllerGroup rightMotors;
    private double leftSpeed;
    private double rightSpeed;
    private static double scaleFactorA = 0.4;
    private static double scaleFactorB = 0.55;
    private static double scaleFactorC = 0.35;
    private static double scaleFactorD = 0.6;
    private static double scaleA = 0.65;
    private static double scaleB = 0.4;
    protected Timer driveTimer;

    public CustomTankDrive(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors){
        driveTimer = new Timer();
        requireNonNull(leftMotors, "leftmotors could not be null");
        requireNonNull(rightMotors, "rightmotors could not be null");
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        SendableRegistry.addChild(this, leftMotors);
        SendableRegistry.addChild(this, rightMotors);
    }
    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);   
    }
    @Override
    public void stopMotor() {
        leftMotors.stopMotor();
        rightMotors.stopMotor();
        feed();
    }
    @Override
    public String getDescription() {    
        return "CustomTankDrive";
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DifferentialDrive");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
    }
    public void drive(double forwardSpeed, double turnSpeed, double speedMultiplier , JoystickScaling scaleType,double curveScaling, DriveStyle driveStyle, boolean driveBack){
        double turnPower = scaleValue(-turnSpeed, scaleType);
        double forwardPower = scaleValue(forwardSpeed, scaleType);
        switch(driveStyle){
            case CUSTOM_TANK:
                
                rightSpeed = (turnPower -(-forwardPower-(turnPower/curveScaling)))/(1+Math.abs(turnPower/curveScaling));
                leftSpeed =(turnPower +(-forwardPower+(turnPower/curveScaling)))/(1+Math.abs(turnPower/curveScaling));
                System.out.println("turnPower"+ turnPower+"forwardPower"+forwardPower);
                System.out.println("rightspeed"+rightSpeed+"leftspeed"+leftSpeed);
                break;
            case ARCADE_TANK:
                double Ls = forwardPower +((!(Math.signum(forwardPower) == 0)?Math.signum(forwardPower):1)*turnPower);
                double Rs = forwardPower -((!(Math.signum(forwardPower) == 0)?Math.signum(forwardPower):1)*turnPower);
                if(forwardPower > 0){
                    leftSpeed = Ls;
                    rightSpeed = forwardPower +((MathUtil.clamp(Ls, -1, 1)-Ls)/2);
                }
                else{
                    leftSpeed = forwardPower +((MathUtil.clamp(Rs, -1, 1)-Rs)/2);
                    rightSpeed = Rs;
                }
                break;
            case IRIS_ARCADE_TANK:
            leftSpeed = scaleFactorA*Math.abs(turnSpeed)+scaleFactorB*turnSpeed*turnSpeed;
            rightSpeed = scaleFactorC*Math.abs(forwardSpeed)+scaleFactorD*forwardSpeed*forwardSpeed;
            
            if (turnSpeed < 0) {
                leftSpeed *= -1;
            }

            if (forwardSpeed < 0) {
                rightSpeed *= -1;
            }

            leftMotors.set(leftSpeed - rightSpeed);
            rightMotors.set(rightSpeed + leftSpeed);
                break;
            case LINEAR_INTERPOLATION_ARCADE:
            if (turnSpeed > 0 && forwardSpeed > 0) {
                leftSpeed = (1-forwardSpeed)*turnSpeed*scaleB+forwardSpeed;
                rightSpeed = (forwardSpeed*(scaleB-scaleA-1)-scaleB)*turnSpeed+forwardSpeed;
            } else if (turnSpeed < 0 && forwardSpeed > 0) {
                leftSpeed = (forwardSpeed*(scaleB-scaleA-1)-scaleB)*turnSpeed+forwardSpeed;
                rightSpeed = (1-forwardSpeed)*turnSpeed*scaleB+forwardSpeed;
            } else if (turnSpeed < 0 && forwardSpeed < 0) {
                leftSpeed = -(1-forwardSpeed)*turnSpeed*scaleB-forwardSpeed;
                rightSpeed = -(forwardSpeed*(scaleB-scaleA-1)-scaleB)*turnSpeed-forwardSpeed;
            } else {
                leftSpeed = -(forwardSpeed*(scaleB-scaleA-1)-scaleB)*turnSpeed-forwardSpeed;
                rightSpeed = -(1-forwardSpeed)*turnSpeed*scaleB-forwardSpeed;
            }


            rightSpeed = forwardPower+turnPower;
            leftSpeed = turnPower-forwardPower;
            rightMotors.set(rightSpeed);
            leftMotors.set(leftSpeed);
            break;
        case SATURATED_ARCADE:
        double saturatedInput;
        double greaterInput = Math.max(Math.abs(turnSpeed), Math.abs(forwardSpeed));
        double smallerInput = Math.min(Math.abs(turnSpeed), Math.abs(forwardSpeed));

        if (greaterInput > 0.0) {
            saturatedInput = smallerInput/greaterInput + 1;
        } else {
            saturatedInput = 1.0;
        }

        turnPower = turnPower/saturatedInput;
        forwardPower = forwardPower/saturatedInput;

        rightSpeed = forwardPower+turnPower;
        leftSpeed = turnPower-forwardPower;
            break;
        default:
            // the arcade drive is the default, we hopefully wont have to use it though
                rightSpeed = forwardPower-turnPower;
                leftSpeed = forwardPower+turnPower;
        }
        rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);
        leftSpeed = MathUtil.clamp(leftSpeed,-1,1);
        rightSpeed *= speedMultiplier;
        leftSpeed *= speedMultiplier;

    if (driveStyle != DriveStyle.LINEAR_INTERPOLATION_ARCADE && driveStyle != DriveStyle.IRIS_ARCADE_TANK) {
        rightMotors.set(rightSpeed);
        leftMotors.set(-leftSpeed);
    }
    
    if (driveBack) {
        driveTimer.reset();
        driveTimer.start();
        while (driveTimer.get() < 0.2) {
            rightMotors.set(0.5);
            leftMotors.set(-0.5);
        }
    }
        feed();
    }

    public double scaleValue(double rawInput, JoystickScaling scaleType){
        double scaledValue = rawInput;
        scaledValue = MathUtil.applyDeadband(scaledValue, DriveConstants.DEFAULT_DEADBAND);
        // note if you have time to put whatever one you are using on top to make the code run faster
        switch(scaleType){
            case SQUARED_EXPONTENTIAL:
                scaledValue *= Math.abs(scaledValue);
                break;
            case CUBIC_EXPONENTIAL:
                scaledValue *= scaledValue*scaledValue;
                // there is absolutely no reason that this code shouldn't work but im still scared
                break;
            case SQUARE_ROOTED:
                scaledValue = (scaledValue >0)?Math.sqrt(scaledValue):-Math.sqrt(scaledValue);
                break;
            case CUBE_ROOTED:
                scaledValue= Math.cbrt(scaledValue);
                break;
            case LOGARITHMIC:
                MathUtil.applyDeadband(scaledValue, .1);
                // idk what this math even is just ignore it and move on with your life
                scaledValue= (scaledValue>0)?(Math.log(scaledValue)/Math.log(Math.E))+1:(Math.log(-scaledValue)/Math.log(Math.E))+1;
                break;
            
            default:
            // default is linear
                break;
        }
        
        MathUtil.clamp(scaledValue, -1, 1);
        return scaledValue;
    }
    public void tankDrive(double leftSpeed, double rightSpeed){

        leftMotors.set(MathUtil.clamp(leftSpeed, -1, 1));
        rightMotors.set(MathUtil.clamp(rightSpeed, -1, 1));
        feed();
    }

}
