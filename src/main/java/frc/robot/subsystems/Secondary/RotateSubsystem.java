// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

public class RotateSubsystem extends SubsystemBase {

    private SparkFlex rotateMotor;
    private AbsoluteEncoder rotateEncoder;
    public SparkClosedLoopController  rotatePID;
    private SparkFlexSim rotateMotorSim;
    private SparkAbsoluteEncoderSim rotateEncoderSim;
    private SparkFlexConfig rotateMtrCfg;
    // private AbsoluteEncoderConfig encCfg;
    // private SoftLimitConfig rotateMtrSftLmtCfg;
    private double kP = 0.01, kI = 0.0, kD = 0.0;//p was 0.0005
    private double kFF = 0.0;
    private double kOutputMin = -1.0;
    private double kOutputMax = 1.0;
    private double kMaxRPM = 5676.0;
    private double kMaxAccel = 20000.0;

    

    public RotateSubsystem() {
        rotateMotor = new SparkFlex(Constants.ArmConstants.ARM_MOTOR_PORT, MotorType.kBrushless);
        rotateMtrCfg = new SparkFlexConfig();
        // encCfg = new AbsoluteEncoderConfig();
        // rotateMtrSftLmtCfg = new SoftLimitConfig();

        rotatePID = rotateMotor.getClosedLoopController();

        rotateEncoder = rotateMotor.getAbsoluteEncoder();
 
        rotateMtrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);
        rotateMtrCfg
            .absoluteEncoder
                .positionConversionFactor(360)
                ;
        rotateMtrCfg
            .softLimit
                .forwardSoftLimit(180.0) 
                .reverseSoftLimit(290.0);
        rotateMtrCfg
            .closedLoop
                .pidf(kP, kI, kD, kFF)
                //.outputRange(kOutputMin, kOutputMax)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .maxMotion
                    .maxAcceleration(kMaxAccel)
                    .maxVelocity(kMaxRPM)
                    .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        rotateMotor.configure(rotateMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // //TODO: Add soft limits
        // rotateMtrSftLmtCfg
        // .forwardSoftLimit(60.0) 
        // .reverseSoftLimit(120.0)
        // .apply(rotateMtrSftLmtCfg);
            
        // Add motors to the simulation
        if (Robot.isSimulation()) {
            rotateMotorSim = new SparkFlexSim(rotateMotor, DCMotor.getNEO(1));
            rotateEncoderSim = new SparkAbsoluteEncoderSim(rotateMotor);
            rotateMotorSim.setPosition(90);
            rotateEncoderSim.setPosition(90);
            rotateMotorSim.setVelocity(0);
            rotateEncoderSim.setVelocity(0);
        }
    }
    
    // // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void setArm(double pos) {
        rotatePID.setReference(pos, SparkFlex.ControlType.kPosition);
        if (Robot.isSimulation()) {
            rotateMotorSim.setPosition(pos);
            rotateEncoderSim.setPosition(pos);
        }


    }
    
    public Command ForwardCmd() {
    return this.run(
        () -> {
            setArm(Constants.ArmConstants.ARM_OUT_POSE);
        });
    }

    public Command MiddleCmd() {
    return this.run(
        () -> {
            setArm(Constants.ArmConstants.ARM_MIDDLE_POSE);
        });
    }

    public Command UpCmd() {
      return this.run(
          () -> {
              setArm(Constants.ArmConstants.ARM_UP_POSE);
          });
      }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        if (Robot.isSimulation()) {
            // rotateEncoderSim.setPosition(rotateMotorSim.getPosition());
            // rotateMotorSim.iterate(rotateEncoderSim.getPosition(), rotateMotorSim.getBusVoltage(),.005);
        }
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Rotate Speed (RPM)", rotateEncoderSim.getPosition());
    } else {
        SmartDashboard.putNumber("Rotate Speed (RPM)", rotateEncoder.getPosition());
    }
    }
}