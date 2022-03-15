package frc.robot.subsystems.abstractSubsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveStyle;
import frc.robot.Constants.JoystickScaling;
public class CustomTankDrive extends RobotDriveBase implements Sendable, AutoCloseable{
    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;
    private double leftSpeed;
    private double rightSpeed;

    public CustomTankDrive(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors){
        // DO NOT USE NULL VALUES FOR MOTORS
        leftMotors = this.leftMotors;
        rightMotors = this.rightMotors;
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
        builder.addDoubleProperty("Left Motor Speed", leftMotors::get, leftMotors::set);
        builder.addDoubleProperty("Right Motor Speed", rightMotors::get, rightMotors::set);    
    }
    public void drive(double turnSpeed, double forwardSpeed, double speedMultiplier , JoystickScaling scaleType,double curveScaling, DriveStyle driveStyle){
        double turnPower = scaleValue(turnSpeed, speedMultiplier, scaleType);
        double forwardPower = scaleValue(forwardSpeed, speedMultiplier, scaleType);
        switch(driveStyle){
            case CUSTOM_TANK:
                rightSpeed = (forwardPower>0)?(forwardPower -(turnPower-(forwardPower/curveScaling)))/1+(forwardPower+curveScaling):(forwardPower -(turnPower-(forwardPower/curveScaling)))/1+(forwardPower-curveScaling);
                leftSpeed = (forwardPower>0)?(forwardPower +(turnPower+(forwardPower/curveScaling)))/1+(forwardPower+curveScaling):(forwardPower +(turnPower+(forwardPower/curveScaling)))/1+(forwardPower-curveScaling);
                break;
            default:
            // the arcade drive is the default, we hopefully wont have to use it though
                rightSpeed = forwardPower-turnPower;
                leftSpeed = forwardPower+turnPower;
        }
        rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);
        leftSpeed = MathUtil.clamp(leftSpeed,-1,1);
        rightMotors.set(rightSpeed);
        leftMotors.set(leftSpeed);
        feed();
    }

    public double scaleValue(double rawInput, double speedMultiplier, JoystickScaling scaleType){
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
                scaledValue= (scaledValue>0)?(Math.log(scaledValue)/Math.E)+1:(Math.log(-scaledValue)/Math.E)+1;
                break;
            
            default:
            // default is linear
                break;
        }
        scaledValue *= speedMultiplier;
        return scaledValue;
    }
    public void tankDrive(double leftSpeed, double rightSpeed){

        leftMotors.set(MathUtil.clamp(leftSpeed, -1, 1));
        rightMotors.set(MathUtil.clamp(rightSpeed, -1, 1));
        feed();
    }

}
