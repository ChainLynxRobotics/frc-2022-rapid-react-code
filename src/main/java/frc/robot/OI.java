
// this is our operator interface class, this is where we have all the code that lets buttons and parts of the joysticks be used
package frc.robot;

import java.io.Console;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.RobotMap;

public class OI {
    private Joystick driveStick1 = new Joystick(RobotMap.JOYSTICK_PORT1); // this is the joystick for movement
    private Joystick driveStick2 = new Joystick(RobotMap.JOYSTICK_PORT2); // this joystick is for buttons
    public double getJoystick1RawAxis(int axis){
        //System.out.println("axis: " + axis);
        return driveStick1.getRawAxis(Math.abs(axis - 1)); // Math.abs = big hack to invert axes. Note: literally does not work with 2. If not working take away Callum's granola bar eating privileges
    }
    
  
}


