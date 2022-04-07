
// this is our operator interface class, this is where we have all the code that lets buttons and parts of the joysticks be used
package frc.robot;

//import java.io.Console;
//import edu.wpi.first.wpilibj.Joystick;
// i changed it to a generic HID instead of joystick to make code easier to compare to docs
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.RobotMap;

// ask whoever the operator ends up being what button they want the intake to be
public class OI {
    private GenericHID driveStick = new GenericHID(RobotMap.JOYSTICK_PORT1); // this is the joystick for movement
    private GenericHID operatorStick = new GenericHID(RobotMap.JOYSTICK_PORT2); // this joystick is for buttons

    private boolean operatorButtons67LastValue = true;
    public double getDriveStickRawAxis(int axis){
        //System.out.println("axis: " + axis);
        return -driveStick.getRawAxis(axis); // Math.abs = big hack to invert axes. Note: literally does not work with 2. If not working take away Callum's granola bar eating privileges
        // note math to invert axis totally uneccessary, just change it in robot container
    }
    
    public double getOperatorStickAxis(int axis){
        return operatorStick.getRawAxis(axis);
    }
    //lmao the axis -1 needed to make this code work is already there in the atrocity that is the getdrivestickrawaxis method
    public int getDriveStickSliderAxis(){
        return driveStick.getAxisCount()-1 ;
    }
    public int getOperatorStickSliderAxis(){
        return operatorStick.getAxisCount()-1;
    }

    public boolean lowerHubShoot() {
        if(operatorStick.getRawButtonPressed(10)) {
            return true;
        } else {
            return false;
        }
    }
    // i hate everything about this piece of code but i am not going to declare classwide variables for every button so i will just cry
    public boolean getOperatorButtons67Toggle(){
        if(operatorStick.getRawButtonPressed(6)){
            operatorButtons67LastValue = true;
        }
        else if(operatorStick.getRawButtonPressed(7)){
            operatorButtons67LastValue = false;
        }
        return operatorButtons67LastValue;
    }
    
    public boolean getOperatorButton(int button){
        return operatorStick.getRawButton(button);
    }
    public boolean getDriverButton(int button){
        return driveStick.getRawButton(button);
    }
}


