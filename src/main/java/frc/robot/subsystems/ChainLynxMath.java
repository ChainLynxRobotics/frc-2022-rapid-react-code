import java.util.Math;
import java.lang.Math;
import frc.robot.Constants;
import frc.robot.OI;
import edu.wpi.first.wpilibj.AnalogAccelerometer;


public class ChainLynxMath{
private long startTime, priorErrorTime;
private double P, I, D;
private double lastError, lastErrorTime, sumError, setpoint;
private double t1, t2, t3, p0, v0; //t1, t2, and t3 are time reference points

AnalogAccelerometer accelerometer = new AnalogAccelerometer(0);
accelerometer.setSensitivity(1);
accelerometer.setZero(2); //2V is the 0 voltage
accelerometer.setRange(Accelerometer.Range.8kG); //-8 to 8 G's;
double acceleration = accelerometer.getAcceleration();

    public static getMotorDirectionToTarget(double currentAngle, double targetAngle, boolean isFastest) {
        int forwardOrBackward = (isFastest && (Math.abs(currentAngle-targetAngle) < 180) ? 1 : -1)*(currentAngle-targetAngle < 0 ? -1 : 1);
        if(Math.abs(currentAngle-targetAngle) < 0.01) {
            motor.set(0); //define a motor before using this method
        } else if(forwardOrBackward == -1) {
            motor.set(-1);
        } else {
            motor.set(1);
        }
    }

    public static radiansToDegrees(double angle) {
        angle = angle*180/Math.PI;
        return angle;
    }

    //number of inches traveled per encoder revolution (ticks per rotation*inches per rotation)
    public static double ticksToFeet() {
        double feet = 1.0/128*2*Math.PI*WHEEL_RADIUS;
        return feet;
    }

    public static horizontalDisplacmentToVelocity(double horizontalDiff, double heightDiff, double angle) {
        double velocityInitial = Math.sqrt(((Math.sin(angle))^2-19.6*horizontalDiff*sin(angle/2)+Math.sqrt((19.6*horizontalDiff*sin(angle/2)-(Math.sin(angle/2)^2)^2)-4*(Math.sin(theta/2))^2(96.04*horizontalDiff^2-19.6*heightDiff)))/(2*(Math.sin(angle/2))^2));
        return velocityInitial;
    }

    //variables initialized at the start of class
    private static double getPathTime() {
        return (Timer.getFPGATimestamp() - this.startTime)/1000; 
    }

    private static double getPathPosition(double t) {
        if (t<-this.t1) {
            double t0 = 0;
            return this.p0 + this.getPathSpeed(t0)*t+0.5*this.getPathAcceleration()*t^2;
        } else if (t<=this.t1+this.t2) {
            double t0 = this.t1;
            double dt = t-this.t1;
            return this.getPathPosition(t0)+this.getPathSpeed(t0)*dt+0.5*this.getPathAcceleration()*t^2;
        } else if (t<this.t1+this.t2+this.t3) {
            double t0 = this.t1+this.t2;
            double dt = t-this.t1-this.t2;
            return this.getPathPosition(t0)+this.getPathSpeed(t0)*dt+0.5*this.getPathAcceleration()*t^2;
        } else {
            return this.getPathPosition(this.t1+this.t2+this.t3);
        }
    }

    private static double getPathSpeed(double t) {
        if (t <= this.t1) {
            return this.v0 + this.getPathAcceleration()*t;
        } else if (t <= this.t1+this.t2) {
            return this.getPathSpeed(this.t1)+this.getPathAcceleration()*(t-this.t1);
        } else if (t <= this.t1+this.t2+this.t3) {
            return this.getPathSpeed(this.t1+this.t2)+this.getPathAcceleration()*(t-t1-t2);
        } else {
            return this.getPathSpeed(this.t1+this.t2+this.t3);
        }
    }

    private static double getPathAcceleration() {
        double a;
            if (tLogitechJoystick.getRawAxis(OI.xAxis) == 0 && tLogitechJoystick.getRawAxis(OI.yAxis) != 0) {
                a = accelerometer.getY();
            } else if (tLogitechJoystick.getRawAxis(OI.yAxis) == 0 && tLogitechJoystick.getRawAxis(OI.xAxis) != 0) {
                a = accelerometer.getX();
            } else {
                a = 0;
            }
        return a;
    }
    
    public static getMotorPower(double currentPosition, double currentSpeed) {
        double pathTime = getPathTime();
        double pathPosition = getPathPosition(pathTime);

        double error = currentPosition - pathPosition;
        double errorSum += error;
        double dt = (Timer.getFPGATimestamp - this.lastErrorTime)/1000;
        double timeChange = 0;

        if(this.lastErrorTime != 0 && dt != 0) {
            timeChange = (error-this.lastError)/dt;
        }

        lastError = error;
        this.lastErrorTime = Timer.getFPGATimestamp;
        this.p0 = currentPosition;
        this.v0 = currentVelocity;
        this.setpoint = setpoint;

        pathTime = getPathTime();

        double power = this.P*error + this.I*errorSum + this.D*timeChange; //can also include path speed term


    }

}
