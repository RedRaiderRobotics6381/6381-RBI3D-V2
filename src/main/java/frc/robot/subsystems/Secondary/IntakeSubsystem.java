// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new PincherSubsystem. */
  public SparkFlex intakeMotorLeader;
  public SparkFlex intakeMotorFollower;
  private RelativeEncoder intakeEncoderLeader;
  private RelativeEncoder intakeEncoderFollower;
  public SparkClosedLoopController  intakeLeaderPID;
  public SparkClosedLoopController  intakeFollowerPID;
  public SparkFlexSim intakeMotorLeaderSim;
  public SparkFlexSim intakeMotorFollowerSim;
  public SparkRelativeEncoderSim intakeEncoderLeaderSim;
  public SparkRelativeEncoderSim intakeEncoderFollowerSim;
  private double kLeaderP = 0.0001, kLeaderI = 0.0, kLeaderD = 0.0;
  private double kFollowerP = 0.0001, kFollowerI = 0.0, kFollowerD = 0.0;
  private double kLeaderFF = 0.0, kFollowerFF = 0.0;
  private double kLeaderOutputMin = -1.0, kFollowerOutputMin = -1.0;
  private double kLeaderOutputMax = 1.0, kFollowerOutputMax = 1.0;
  private double kLeaderMaxRPM = 5676, kFollowerMaxRPM = 5676;
  private double kLeaderMaxAccel = 10000, kFollowerMaxAccel = 10000;
  
  public IntakeSubsystem() {
    intakeMotorLeader = new SparkFlex(Constants.IntakeConstants.TOP_INTAKE_MOTOR_PORT, MotorType.kBrushless);
    intakeMotorFollower = new SparkFlex(Constants.IntakeConstants.BOTTOM_INTAKE_MOTOR_PORT, MotorType.kBrushless);
    
    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    SparkFlexConfig followerConfig = new SparkFlexConfig();
    // SoftLimitConfig leaderSoftLimit = new SoftLimitConfig();
    // SoftLimitConfig followerSoftLimit = new SoftLimitConfig();

    intakeLeaderPID = intakeMotorLeader.getClosedLoopController();
    intakeFollowerPID = intakeMotorFollower.getClosedLoopController();

    intakeEncoderLeader = intakeMotorLeader.getEncoder();
    intakeEncoderFollower = intakeMotorFollower.getEncoder();

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
    intakeMotorLeader.configure(leaderConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerConfig
        .follow(intakeMotorLeader, true)
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
    intakeMotorFollower.configure(followerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    if (Robot.isSimulation()) {
      intakeMotorLeaderSim = new SparkFlexSim(intakeMotorLeader, DCMotor.getNEO(1));
      intakeMotorFollowerSim = new SparkFlexSim(intakeMotorFollower, DCMotor.getNEO(1));
      intakeEncoderLeaderSim = new SparkRelativeEncoderSim(intakeMotorLeader);
      intakeEncoderFollowerSim = new SparkRelativeEncoderSim(intakeMotorFollower);
      // leaderIntakeSim.setVelocity(0);
      // followerIntakeSim.setVelocity(0);
      // leaderEncoderSim.setVelocity(0);
      // followerEncoderSim.setVelocity(0);
    }

  }

  @Override
  public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
      if (Robot.isSimulation()) {
          // leaderEncoderSim.setPosition(rotateMotorSim.getPosition());
          intakeMotorLeaderSim.iterate(intakeEncoderLeaderSim.getVelocity(), intakeMotorLeaderSim.getBusVoltage(),.005);
          intakeMotorFollowerSim.iterate(intakeEncoderFollowerSim.getVelocity(), intakeMotorFollowerSim.getBusVoltage(),.005);
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Intake Lead Speed (RPM)", intakeEncoderLeaderSim.getPosition());
        SmartDashboard.putNumber("Intake Follower Speed (RPM)", intakeEncoderFollowerSim.getPosition());
    } else {
        SmartDashboard.putNumber("Intake Lead Speed (RPM)", intakeEncoderLeader.getPosition());
        SmartDashboard.putNumber("Intake Follower Speed (RPM)", intakeEncoderFollower.getPosition());
    }
  }
  

  public void runIntake(double speed) {
    intakeLeaderPID.setReference(speed, SparkFlex.ControlType.kMAXMotionVelocityControl);
  }

  public Command IntakeCmd() {
    return this.runEnd(
        () -> {
            runIntake(Constants.IntakeConstants.INTAKE_SPEED);
        },
        () -> {
            runIntake(Constants.IntakeConstants.HOLD_SPEED);
        }
      );
  }

  public Command OuttakeCmd() {
    return this.runEnd(
        () -> {
            runIntake(Constants.IntakeConstants.OUTTAKE_SPEED);
        },
        () -> {
            runIntake(Constants.IntakeConstants.STOP_SPEED);
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




