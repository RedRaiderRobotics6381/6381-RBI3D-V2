// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
// import com.revrobotics.spark.config.SoftLimitConfig;
//import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.revrobotics.servohub.ServoHub;
// import com.revrobotics.servohub.config.ServoHubConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new PincherSubsystem. */
  public SparkFlex leaderIntake;
  public SparkFlex followerIntake;
  private RelativeEncoder leaderEncoder;
  private RelativeEncoder followerEncoder;
  public SparkClosedLoopController  leaderPIDController;
  public SparkClosedLoopController  followerPIDController;
  public SparkFlexSim leaderIntakeSim;
  public SparkFlexSim followerIntakeSim;
  public SparkRelativeEncoderSim leaderEncoderSim;
  public SparkRelativeEncoderSim followerEncoderSim;
  private double kLeaderP = 0.0005, kLeaderI = 0.0, kLeaderD = 0.0;
  private double kFollowerP = 0.0005, kFollowerI = 0.0, kFollowerD = 0.0;
  private double kLeaderFF = 0.0005, kFollowerFF = 0.0005;
  private double kLeaderOutputMin = -1.0, kFollowerOutputMin = -1.0;
  private double kLeaderOutputMax = 1.0, kFollowerOutputMax = 1.0;
  private double kLeaderMaxRPM = 5676, kFollowerMaxRPM = 5676;
  private double kLeaderMaxAccel = 10000, kFollowerMaxAccel = 10000;
  
  public IntakeSubsystem() {
        leaderIntake = new SparkFlex(Constants.IntakeConstants.LEFT_INTAKE_MOTOR_PORT, MotorType.kBrushless);
        followerIntake = new SparkFlex(Constants.IntakeConstants.RIGHT_INTAKE_MOTOR_PORT, MotorType.kBrushless);
        
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        SparkFlexConfig followerConfig = new SparkFlexConfig();
        // SoftLimitConfig leaderSoftLimit = new SoftLimitConfig();
        // SoftLimitConfig followerSoftLimit = new SoftLimitConfig();

        leaderPIDController = leaderIntake.getClosedLoopController();
        followerPIDController = followerIntake.getClosedLoopController();

        leaderEncoder = leaderIntake.getEncoder();
        followerEncoder = followerIntake.getEncoder();

        leaderConfig
            .inverted(false)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .pidf(kLeaderP, kLeaderI, kLeaderD, kLeaderFF)
                .outputRange(kLeaderOutputMin, kLeaderOutputMax)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                    .maxAcceleration(kLeaderMaxAccel)
                    .maxVelocity(kLeaderMaxRPM);
        leaderIntake.configure(leaderConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerConfig
            .follow(leaderIntake, true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .pidf(kFollowerP, kFollowerI, kFollowerD, kFollowerFF)
                .outputRange(kFollowerOutputMin, kFollowerOutputMax)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                    .maxAcceleration(kFollowerMaxAccel)
                    .maxVelocity(kFollowerMaxRPM);
        followerIntake.configure(followerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Intake Lead Speed (RPM)", leaderEncoderSim.getPosition());
        SmartDashboard.putNumber("Intake Follower Speed (RPM)", followerEncoderSim.getPosition());
    } else {
        SmartDashboard.putNumber("Intake Lead Speed (RPM)", leaderEncoder.getPosition());
        SmartDashboard.putNumber("Intake Follower Speed (RPM)", followerEncoder.getPosition());
    }
  }
  

  public void runIntake(double speed) {
    leaderPIDController.setReference(speed, SparkFlex.ControlType.kMAXMotionVelocityControl);
  }

  public Command IntakeCmd() {
    return this.runOnce(
        () -> {
            runIntake(Constants.IntakeConstants.INTAKE_SPEED);
        }
      );
  }

  public Command OuttakeCmd() {
    return this.runOnce(
        () -> {
            runIntake(Constants.IntakeConstants.OUTTAKE_SPEED);
        }
      );
  }

  public Command StopCmd() {
    return this.runOnce(
        () -> {
            runIntake(Constants.IntakeConstants.STOP_SPEED);
        }
      );
  }}
//   public void setPinchAngle(double angle){ {
//     pincherServo.setAngle(angle);
//   }
//  }
//  public Command PincherRotateCmd(double desiredAngle) {
//   return this.runOnce(
//       () -> {
//           // rotateMotorL.set(-0.25);
//           // feederLauncher.set(-0.25);
//           //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
//           setPinchAngle(desiredAngle);
//       }
//     );
//   }}




