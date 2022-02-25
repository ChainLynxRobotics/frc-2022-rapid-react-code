package frc.robot.subsystems;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.constants;
import java.util.Math;

//all distance measurements are in meters 
//high goal
public class shootingHashMaps implements shooter{
    static double increment = 0.01;
    public static distanceToVelocity(double range) {
        shooter shooterVelocity = new shooter();
        double rangeInit = shooterVelocity.range;
        HashMap<double, double> convert = new HashMap<>();
        for(int i = 0; i < 289; i++) {
            double rangeCurrent = rangeInit + 0.01*i;
            convert.set(rangeCurrent+1.59, Math.sqrt(-0.0591+Math.sqrt(0.0035+0.00496(rangeCurrent+1.59)^2)/0.00248));
        }
    }
}

//in shooter class: convert.get(range+1.59) to get corresponding velocity
