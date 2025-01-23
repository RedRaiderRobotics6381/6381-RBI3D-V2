// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static double Max_Speed_Multiplier = 0.75;

  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;

  }
  
  public static class ElevatorConstants {
    public static final int LEFT_ELEVATOR_MOTOR_PORT = 9;
    public static final int RIGHT_ELEVATOR_MOTOR_PORT = 13;
    public static final int LEFT_ELEVATOR_MOTOR_PORT = 9;
    public static final int RIGHT_ELEVATOR_MOTOR_PORT = 13;

    public static final int START_POSE = 0;
    public static final int TROUGH_POSE = 0;
    public static final int REEF_LOW_POSE = 4;
    public static final int REEF_MIDDLE_POSE = 8;
    public static final int REEF_HIGH_POSE = 0;
    public static final int ALGAE_SCORE_POSE = 0;
    public static final int ALGAE_PICKUP_POSE = 0;
    public static final int HUMAN_PLAYER_POSE = 0;

  }

  public static class ArmConstants {
    public static final int ARM_MOTOR_PORT = 12;
    public static final int ARM_OUT_POSE = 200;
    public static final int ARM_UP_POSE = 230;
    public static final int ARM_MIDDLE_POSE = 260;
  }

  // public static class ServoConstants {

  //   public static final int PINCHER_SERVO_PORT_1 = 1;
  //   public static final int PINCHER_SERVO_PORT_2 = 2;
  //   public static final int CLOSED_ANGLE = 90;
  //   public static final int OPEN_ANGLE = 0;

  // }

  public static class IntakeConstants {
    public static final int LEFT_INTAKE_MOTOR_PORT = 10;
    public static final int RIGHT_INTAKE_MOTOR_PORT = 11;

    public static final double INTAKE_SPEED = 500;
    public static final double OUTTAKE_SPEED = -500;
    public static final double STOP_SPEED = 0;
    public static final double HOLD_SPEED = 100;
  }
}
