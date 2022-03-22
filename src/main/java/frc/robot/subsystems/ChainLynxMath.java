import java.lang.Math;
import java.util.Timer;

import javax.management.monitor.Monitor;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.DriveConstants;

public class ChainLynxMath{
private long startTime, priorErrorTime;
private double P, I, D;
private double lastError, lastErrorTime, sumError, setpoint;
private double t1, t2, t3, p0, v0; //t1, t2, and t3 are time reference points

public 

/*
AnalogAccelerometer accelerometer = new AnalogAccelerometer(0);
accelerometer.setSensitivity(1);
accelerometer.setZero(2); //2V is the 0 voltage
accelerometer.setRange(Accelerometer.Range.8kG); //-8 to 8 G's;
double acceleration = accelerometer.getAcceleration();
*/
    public static void getMotorDirectionToTarget(double currentAngle, double targetAngle, boolean isFastest) {
        int forwardOrBackward = (isFastest && (Math.abs(currentAngle-targetAngle) < 180) ? 1 : -1)*(currentAngle-targetAngle < 0 ? -1 : 1);
        if(Math.abs(currentAngle-targetAngle) < 0.01) {
            Monitor.set(0); //define a motor before using this method
        } else if(forwardOrBackward == -1) {
            motor.set(-1);
        } else {
            motor.set(1);
        }
    }

    public double radiansToDegrees(double angle) {
        angle = angle*180/Math.PI;
        return angle;
    }

    //number of inches traveled per encoder revolution (ticks per rotation*inches per rotation)
    public static double ticksToFeet() {
        double feet = 1.0/128*2*Math.PI*DriveConstants.WHEEL_RADIUS;
        return feet;
    }

    //for the shooter we aren't going to have this season...
    public static double horizontalDisplacmentToVelocity(double horizontalDiff, double heightDiff, double angle) {
        double velocityInitial = Math.sqrt(((Math.sin(angle))^2-19.6*horizontalDiff*Math.sin(angle/2)+
        Math.sqrt((19.6*horizontalDiff*Math.sin(angle/2)-(Math.sin(angle/2)^2)^2)-4*(Math.sin(angle/2)*(Math.sin(angle/2))
        (96.04*horizontalDiff*horizontalDiff-19.6*heightDiff)))/(2*(Math.sin(angle/2))*(Math.sin(angle/2)));
        return velocityInitial;
    }

    //variables initialized at the start of class
    private double getPathTime() {
        return (Timer.get() - this.startTime)/1000; 
    }

    private double getPathPosition(double t) {
        if (t<-this.t1) {
            double t0 = 0;
            return this.p0 + this.getPathSpeed(t0)*t+0.5*ChainLynxMath.getPathAcceleration()*t*t;
        } else if (t<=this.t1+this.t2) {
            double t0 = this.t1;
            double dt = t-this.t1;
            return this.getPathPosition(t0)+this.getPathSpeed(t0)*dt+0.5*ChainLynxMath.getPathAcceleration()*t*t;
        } else if (t<this.t1+this.t2+this.t3) {
            double t0 = this.t1+this.t2;
            double dt = t-this.t1-this.t2;
            return this.getPathPosition(t0)+this.getPathSpeed(t0)*dt+0.5*ChainLynxMath.getPathAcceleration()*t*t;
        } else {
            return this.getPathPosition(this.t1+this.t2+this.t3);
        }
    }

    private double getPathSpeed(double t) {
        if (t <= this.t1) {
            return this.v0 + ChainLynxMath.getPathAcceleration()*t;
        } else if (t <= this.t1+this.t2) {
            return this.getPathSpeed(this.t1)+ChainLynxMath.getPathAcceleration()*(t-this.t1);
        } else if (t <= this.t1+this.t2+this.t3) {
            return this.getPathSpeed(this.t1+this.t2)+ChainLynxMath.getPathAcceleration()*(t-t1-t2);
        } else {
            return this.getPathSpeed(this.t1+this.t2+this.t3);
        }
    }

    private static double getPathAcceleration() {
        double a;
        //Timer.getFPGATimestamp is in microseconds
        //need to create actual OI and joystick instances
        double dert = (Timer.get()/1000 - lastTime)/1000;
            if (tLogitechJoystick.getRawAxis(OI.xAxis) == 0 && tLogitechJoystick.getRawAxis(OI.yAxis) != 0) {
                //a = accelerometer.getY();
                a = (0.5*m_leftDriveFront.getEncoder().getVelocity()+0.5*m_leftDriveBack.getEncoder().getVelocity())/dert;
            } else if (tLogitechJoystick.getRawAxis(OI.yAxis) == 0 && tLogitechJoystick.getRawAxis(OI.xAxis) != 0) {
                //a = accelerometer.getX();
                a = (0.5*m_rightDriveFront.getEncoder().getVelocity()+0.5*m_rightDriveBack.getEncoder().getVelocity())/dert;
            } else {
                a = 0;
            }
        lastErrorTime = Timer.getFPGATimestamp();
        return a;
    }
    
    public void getMotorPower(double currentPosition, double currentSpeed) {
        double pathTime = getPathTime();
        double pathPosition = getPathPosition(pathTime);

        double error = currentPosition - pathPosition;
        double errorSum = errorSum + error;
        double dt = (Timer.get()/1000 - this.lastErrorTime);
        double timeChange = 0;

        if(this.lastErrorTime != 0 && dt != 0) {
            timeChange = (error-this.lastError)/dt;
        }

        lastError = error;
        this.lastErrorTime = Timer.get();
        this.p0 = currentPosition;
        this.v0 = currentVelocity;
        this.setpoint = setpoint;

        pathTime = getPathTime();

        double power = this.P*error + this.I*errorSum + this.D*timeChange; //can also include path speed term

    }
}