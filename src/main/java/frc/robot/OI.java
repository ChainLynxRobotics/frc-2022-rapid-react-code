
// this is our operator interface class, this is where we have all the code that lets buttons and parts of the joysticks be used
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.RobotMap;

public class OI {
    private Joystick driveStick1 = new Joystick(RobotMap.JOYSTICK_PORT1); // this is the joystick for movement
    private Joystick driveStick2 = new Joystick(RobotMap.JOYSTICK_PORT2); // this joystick is for buttons
    public double getJoystick1RawAxis(int axis){
        return driveStick1.getRawAxis(axis);
    }
    
  
}


