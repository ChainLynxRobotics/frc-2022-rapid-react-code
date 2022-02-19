
// this is our operator interface class, this is where we have all the code that lets buttons and parts of the joysticks be used
package frc.robot;

//import java.io.Console;
//import edu.wpi.first.wpilibj.Joystick;
// i changed it to a generic HID instead of joystick to make code easier to compare to docs
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.RobotMap;

// ask whoever the operator ends up being what button they want the intake to be
public class OI {
    private GenericHID driveStick = new GenericHID(RobotMap.JOYSTICK_PORT1); // this is the joystick for movement
    private GenericHID operatorStick = new GenericHID(RobotMap.JOYSTICK_PORT2); // this joystick is for buttons
    private JoystickButton operatorButton2 = new JoystickButton(operatorStick, 1);
    
    
    public double getDriveStickRawAxis(int axis){
        //System.out.println("axis: " + axis);
        return driveStick.getRawAxis(Math.abs(axis - 1)); // Math.abs = big hack to invert axes. Note: literally does not work with 2. If not working take away Callum's granola bar eating privileges
    }
    
    public double getOperatorStickAxis(int axis){
        return operatorStick.getRawAxis(axis);
    }
    public int getDriveStickSliderAxis(){
        return driveStick.getAxisCount() -1;
    }
    // this is the code of a broken programmer
    public JoystickButton getOperatorButton2(){
        return operatorButton2;
    }
    public boolean getOperatorButton(int button){
        return operatorStick.getRawButton(button);
    }
    public boolean getDriverButton(int button){
        return driveStick.getRawButton(button);
    }
}


