package frc.robot.subsystems.abstractSubsystems;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveStyle;
import frc.robot.Constants.JoystickScaling;


//arcade controls integrated with tank drive math (maps joystick positions to x (turning) and y axis (position) speeds)
public class ArcadeTankDrive extends RobotDriveBase implements Sendable, AutoCloseable {
    private final MotorControllerGroup leftMotors;
    private final MotorControllerGroup rightMotors;
    private double leftSpeed;
    private double rightSpeed;
    private double scaleFactorA = 0.4;
    private double scaleFactorB = 0.55;
    private double scaleFactorC = 0.35;
    private double scaleFactorD = 0.6;
    private double scaleA = 0.65;
    private double scaleB = 0.4;

    public ArcadeTankDrive(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors) {
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
        return "ArcadeTankDrive";
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DifferentialDrive");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
    }


//create arcadeTankDrive method (maps joystick positions to motor power, (xaxis, yaxis) -> (turnSpeed, positionSpeed))
    public void drive(double turnSpeed, double forwardSpeed, double speedMultiplier, JoystickScaling scaleType, double curveScaling, DriveStyle driveStyle) {
        double turnPower = scaleValue(turnSpeed, scaleType);
        double forwardPower = scaleValue(forwardSpeed, scaleType);

        if (driveStyle == DriveStyle.CUSTOM_TANK) {
            
            rightSpeed = (turnPower -(-forwardPower-(turnPower/curveScaling)))/(1+Math.abs(turnPower/curveScaling));
            leftSpeed =(turnPower +(-forwardPower+(turnPower/curveScaling)))/(1+Math.abs(turnPower/curveScaling));

            rightSpeed *= speedMultiplier;
            leftSpeed *= speedMultiplier;
            rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);
            leftSpeed = MathUtil.clamp(leftSpeed,-1,1);
        } else if (driveStyle == DriveStyle.ARCADE_TANK) {

            leftSpeed = scaleFactorA*Math.abs(turnPower)+scaleFactorB*turnPower*turnPower;
            rightSpeed = scaleFactorC*Math.abs(forwardPower)+scaleFactorD*forwardPower*forwardPower;

            if (turnPower < 0) {
                leftSpeed *= -1;
            }

            if (forwardPower < 0) {
                rightSpeed *= -1;
            }

            leftMotors.set(rightSpeed + leftSpeed);
            rightMotors.set(rightSpeed - leftSpeed);

        } else if (driveStyle == DriveStyle.SATURATED_ARCADE) { 

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

            leftSpeed = forwardPower+turnPower;
            rightSpeed = forwardPower-turnPower;

        //default drive is arcade derived via linear interpolation 
        } else { 
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

        }

        if (driveStyle != DriveStyle.ARCADE_TANK) {
            leftMotors.set(leftSpeed);
            rightMotors.set(rightSpeed);
        }
        feed();
    }

    //scales joystick motor power inputs
    public double scaleValue(double rawInput, JoystickScaling scaleType) {
        double scaledValue = rawInput;
        scaledValue = MathUtil.applyDeadband(scaledValue, DriveConstants.DEFAULT_DEADBAND);
        if (scaleType == JoystickScaling.SQUARED_EXPONENTIAL) {
            scaledValue *= Math.abs(scaledValue);
        } else if (scaleType ==JoystickScaling.CUBIC_EXPONENTIAL) {
            scaledValue *= scaledValue*scaledValue;
        } else if (scaleType == JoystickScaling.SQUARE_ROOTED) {
            scaledValue = Math.sqrt(Math.abs(scaledValue));
        } else if (scaleType == JoystickScaling.CUBE_ROOTED) {
            scaledValue = Math.cbrt(Math.abs(scaledValue));
        } else if (scaleType == JoystickScaling.LOGARITHMIC) {
            scaledValue = (scaledValue != 0) ? (Math.log(Math.abs(scaledValue))/Math.log(Math.E)) : 0;
        }

        MathUtil.clamp(scaledValue, -1, 1); //set bounds for scaledValue
        return scaledValue;
    }

    public void arcadeTankDrive(double leftSpeed, double rightSpeed) {
        leftMotors.set(leftSpeed);
        rightMotors.set(rightSpeed);
        feed();
    }
}
