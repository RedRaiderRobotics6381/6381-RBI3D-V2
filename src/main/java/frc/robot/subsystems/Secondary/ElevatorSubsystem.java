// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Secondary;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
// import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.EncoderConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {

    public SparkFlex elevMtrLdr;
    public SparkFlex elevMtrFlw;
    private RelativeEncoder elevEncLdr;
    private RelativeEncoder elevEncFlw;
    public SparkClosedLoopController  elevPIDLdr;
    public SparkClosedLoopController  elevPIDFlw;
    public SparkFlexSim elevMtrLdrSim;
    public SparkFlexSim elevMtrFlwSim;
    public SparkRelativeEncoderSim elevEncLdrSim;
    public SparkRelativeEncoderSim elevEncFlwSim;
    private double kLdrP = 0.0005, kLdrI = 0.0, kLdrD = 0.0;
    private double kFlwP = 0.0005, kFlwI = 0.0, kFlwD = 0.0;
    private double kLdrFF = 0.0005, kFlwFF = 0.0005;
    private double kLdrOutputMin = -1.0, kFlwOutputMin = -1.0;
    private double kLdrOutputMax = 1.0, kFlwOutputMax = 1.0;
    private double kLdrMaxRPM = 5676, kFlwMaxRPM = 5676;
    private double kLdrMaxAccel = 10000, kFlwMaxAccel = 10000;
    public DigitalInput limitSwL;
    public DigitalInput limitSwR;
    

    public ElevatorSubsystem() {
        elevMtrLdr = new SparkFlex(Constants.ElevatorConstants.LEFT_ELEVATOR_MOTOR_PORT, MotorType.kBrushless);
        elevMtrFlw = new SparkFlex(Constants.ElevatorConstants.RIGHT_ELEVATOR_MOTOR_PORT, MotorType.kBrushless);

        SparkFlexConfig ldrCfg = new SparkFlexConfig();
        SparkFlexConfig flwCfg = new SparkFlexConfig();
        // EncoderConfig encoderConfig = new EncoderConfig();
        // SoftLimitConfig leaderSoftLimit = new SoftLimitConfig();
        // SoftLimitConfig followerSoftLimit = new SoftLimitConfig();

        elevPIDLdr = elevMtrLdr.getClosedLoopController();
        elevPIDFlw = elevMtrFlw.getClosedLoopController();

        elevEncLdr = elevMtrLdr.getEncoder();
        elevEncFlw = elevMtrFlw.getEncoder();

        ldrCfg
            .inverted(false)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake);
        ldrCfg
            .encoder
                .positionConversionFactor(.0854); //confirm conversion factor
        ldrCfg
            .softLimit
                .forwardSoftLimit(18) 
                .reverseSoftLimit(0);
        ldrCfg
            .closedLoop
                .pidf(kLdrP, kLdrI, kLdrD, kLdrFF)
                .outputRange(kLdrOutputMin, kLdrOutputMax)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                    .maxAcceleration(kLdrMaxAccel)
                    .maxVelocity(kLdrMaxRPM);
        elevMtrLdr.configure(ldrCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        flwCfg
            .follow(elevMtrLdr, true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake);
        flwCfg
            .encoder
                .positionConversionFactor(.0854); //confirm conversion factor
        flwCfg
            .softLimit
                .forwardSoftLimit(18) 
                .reverseSoftLimit(0);
        flwCfg
            .closedLoop
                .pidf(kFlwP, kFlwI, kFlwD, kFlwFF)
                .outputRange(kFlwOutputMin, kFlwOutputMax)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                    .maxAcceleration(kFlwMaxAccel)
                    .maxVelocity(kFlwMaxRPM);
        elevMtrFlw.configure(flwCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // //TODO: Add soft limits
        // followerSoftLimit
        // .forwardSoftLimit(18) 
        // .reverseSoftLimit(0)
        // .apply(followerSoftLimit);

        // Add motors to the simulation
        if (Robot.isSimulation()) {
            elevMtrLdrSim = new SparkFlexSim(elevMtrLdr, DCMotor.getNEO(1));
            elevMtrFlwSim = new SparkFlexSim(elevMtrFlw, DCMotor.getNEO(1));
            elevEncLdrSim = new SparkRelativeEncoderSim(elevMtrLdr);
            elevEncFlwSim = new SparkRelativeEncoderSim(elevMtrFlw);
            elevMtrLdrSim.setPosition(0);
            elevMtrFlwSim.setPosition(0);
            elevEncLdrSim.setVelocity(0);
            elevEncFlwSim.setVelocity(0);

        }
    }
    
    // // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void setElevatorHeight(double pos) {
        // leaderElevatorL.set(speed);
        elevPIDLdr.setReference(pos, SparkMax.ControlType.kMAXMotionPositionControl);
        if (Robot.isSimulation()) {
            // leaderElevatorSim.setVelocity(speed);
            // followerElevatorSim.setVelocity(speed);
            elevPIDFlw.setReference(pos, SparkMax.ControlType.kMAXMotionPositionControl);
        }
    }
    
    public Command ElevatorPosCmd(double position) {
        return this.run(
            () -> {
                // rotateMotorL.set(-0.25);
                // feederLauncher.set(-0.25);
                //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
                if(limitSwL.get() && limitSwR.get() == false){
                    setElevatorHeight(position);
            }}
          );
        }
    // public Command TroughPoseCmd() {
    //     return this.run(
    //         () -> {
    //             // rotateMotorL.set(-0.25);
    //             // feederLauncher.set(-0.25);
    //             //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
    //             setElevatorHeight(Constants.ElevatorConstants.TROUGH_POSE);
    //         }
    //       );
    //     }
    // public Command ReefLowPoseCmd() {
    //     return this.run(
    //         () -> {
    //             // rotateMotorL.set(-0.25);
    //             // feederLauncher.set(-0.25);
    //             //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
    //             setElevatorHeight(Constants.ElevatorConstants.REEF_LOW_POSE);
    //         }
    //       );
    //     }
    // public Command ReefMiddlePoseCmd() {
    //     return this.run(
    //         () -> {
    //             // rotateMotorL.set(-0.25);
    //             // feederLauncher.set(-0.25);
    //             //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
    //             setElevatorHeight(Constants.ElevatorConstants.REEF_MIDDLE_POSE);
    //         }
    //       );
    //     }
    // public Command ReefHighPoseCmd() {
    //     return this.run(
    //         () -> {
    //             // rotateMotorL.set(-0.25);
    //             // feederLauncher.set(-0.25);
    //             //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
    //             setElevatorHeight(Constants.ElevatorConstants.REEF_HIGH_POSE);
    //         }
    //       );
    //     }
    // public Command AlgaeScorePoseCmd() {
    //     return this.run(
    //         () -> {
    //             // rotateMotorL.set(-0.25);
    //             // feederLauncher.set(-0.25);
    //             //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
    //             setElevatorHeight(Constants.ElevatorConstants.ALGAE_SCORE_POSE);
    //         }
    //       );
    //     }
    // public Command AlgaePickUpPoseCmd() {
    //     return this.run(
    //         () -> {
    //             // rotateMotorL.set(-0.25);
    //             // feederLauncher.set(-0.25);
    //             //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
    //             setElevatorHeight(Constants.ElevatorConstants.ALGAE_PICKUP_POSE);
    //         }
    //       );
    //     }
    // public Command HumanPlayerPoseCmd() {
    //     return this.run(
    //         () -> {
    //             // rotateMotorL.set(-0.25);
    //             // feederLauncher.set(-0.25);
    //             //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
    //             setElevatorHeight(Constants.ElevatorConstants.HUMAN_PLAYER_POSE);
    //         }
    //       );
    //     }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        // if (Robot.isSimulation()) {
        //     elevEncLdrSim.setPosition(elevMtrLdrSim.getPosition());
        //     elevEncFlwSim.setPosition(elevMtrFlwSim.getPosition());
        //     elevMtrLdrSim.iterate(elevEncLdrSim.getPosition(), elevMtrLdrSim.getBusVoltage(),.005);
        //     elevMtrFlwSim.iterate(elevEncFlwSim.getPosition(), elevMtrFlwSim.getBusVoltage(),.005);
        // }
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Elevator Lead Speed (RPM)", elevEncLdrSim.getPosition());
        SmartDashboard.putNumber("Elevator Follower Speed (RPM)", elevEncFlwSim.getPosition());
    } else {
        SmartDashboard.putNumber("Elevator Lead Speed (RPM)", elevEncLdr.getPosition());
        SmartDashboard.putNumber("Elevator Follower Speed (RPM)", elevEncFlw.getPosition());
    }
    }
}