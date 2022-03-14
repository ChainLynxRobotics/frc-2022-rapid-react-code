// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
   public static final class RobotMap{

      //these ID's are accurate to the old robot, not the 2022 robot
      
      public static final int MOTOR_LEFT_MASTER_ID = 2; //motor port values are the same as encoder port values
      public static final int MOTOR_LEFT_SLAVE_ID = 1;
      public static final int MOTOR_RIGHT_MASTER_ID = 3;
      public static final int MOTOR_RIGHT_SLAVE_ID = 4;
      public static final boolean LEFT_SIDE_INVERTED = false;
      public static final boolean RIGHT_SIDE_INVERTED = true;
      
      // joystick port, may change if there is a mouse or something plugged in, be careful about that
      public static final int JOYSTICK_PORT1 = 0;
	   public static final int JOYSTICK_PORT2 = 1;
      public static final int BALLHANDLER_MOTOR_ID = 6;
      public static final int ROBOT_ARM_MOTOR_ID = 5;

   }
   public static final class SimulationConstants{
      // these are parameters that are necessary for simulating the drivetrain, more accurate numbers means the simulation is more accurate

	   public static final DCMotor MOTOR_QUANTITY = DCMotor.getNEO(2); //we chave 2 NEO motors on each side of the robot
      public static final double GEAR_RATIO = 10.71; // we have a gear ratio of 5.85:1
	   public static final double MOMENT_OF_INERTIA = 5;// the units for this are kgm^2 and we currently do not have accurate numbers for them so if you need accurate numbers nag design team
      public static final double DRIVETRAIN_WEIGHT = 5.1; //weight of the drivetrain in kg
	   
	   public static final Matrix<N7, N1> MEASUREMENT_NOISE = null; // this is for measurement noise so important if we want to be really accurate in our simulations
      
      
      
      
      public static final boolean SIM_GYRO_INVERTED = true; // this is here because i am not sure if we will get a real gyro or what will happen to any gyro in the simulation
      public static final double VOLTS_SCNDS_SQUARED_PER_METER = .2;
      public static final double VOLTS_SCNDS_PER_METER = 1.98;
      public static final double VOLTS = .22;


   }
   public static final class DriveConstants{

      public static final double WHEEL_RADIUS = Units.inchesToMeters(3); //our wheels have a radius of 3 inches
      public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;
      public static final double ENCODER_PULSE_DISTANCE (int ENCODER_CPR){
         return (WHEEL_RADIUS * Math.PI)/ ENCODER_CPR; 
      }
      public static final double TRACK_WIDTH = Units.inchesToMeters(33); // there are .58 meters between the left and right wheels
      public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
      public static final double AUTO_VOLTAGE_CONSTRAINT = 7;
      public static final double AUTO_DRIVE_SPEED = 0;


      // Robot arm math stuff
      public static final double ROBOT_ARM_EXPONENT_WEIGHT = 1.5;
      public static final double ROBOT_ARM_DELTA_SENSITIVITY = 20;

   } 
   public enum JoystickScaling{
      SQUARED_EXPONTENTIAL,
      CUBIC_EXPONENTIAL,
      LINEAR,
      SQUARE_ROOTED,
      CUBE_ROOTED,
      LOGARITHMIC
   }
   public enum DriveStyle{
      CUSTOM_TANK,
      NORMAL_ARCADE
   }
}
