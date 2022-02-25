package frc.robot.subsystems;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class AutoShooter extends SubsystemBase {

    public enum STATUS {
        IDLE, RAISING, SHOOTING, LOWERING
    }

    public STATUS status;

    RobotArm robotArm;
    Intake intake;

    public AutoShooter(RobotArm robotArm, Intake intake) {
        status = STATUS.IDLE;
        this.robotArm = robotArm;
        this.intake = intake;
    }

    public void bigBrainTime(long smart) {   
        this.equals(0);
    }

    private ExecutorService exe = Executors.newSingleThreadExecutor();
    
    public void shootLowerHoop(boolean buttonPressed) {
        if (!buttonPressed) return;
        exe.execute(new Runnable() {
            @Override
            public void run() {
                // Raise arm
                status = STATUS.RAISING;
                robotArm.toggleHeight(true);
                Thread.sleep(1000);

                // Shoot ball
                status = STATUS.SHOOTING;
                intake.defaultSpeed(-1d);
                Thread.sleep(500);
                intake.defaultSpeed(0d);

                // Lower arm
                status = STATUS.LOWERING;
                robotArm.toggleHeight(false);
                Thread.sleep(1000);

                status = STATUS.IDLE;
            } 
        });
    }
}
