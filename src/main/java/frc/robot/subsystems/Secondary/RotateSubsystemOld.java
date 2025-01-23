// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// package frc.robot.subsystems.Secondary;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// import frc.robot.Robot;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.config.AbsoluteEncoderConfig;
// import com.revrobotics.spark.config.SoftLimitConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.sim.SparkAbsoluteEncoderSim;
// import com.revrobotics.sim.SparkFlexSim;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkFlex;

// public class RotateSubsystem extends SubsystemBase {

//     public SparkFlex rotateMotor;
//     private AbsoluteEncoder rotateEncoder;
//     public SparkClosedLoopController  rotatePIDController;
//     public SparkFlexSim rotateMotorSim;
//     public SparkAbsoluteEncoderSim rotateEncoderSim;
//     private double kLeaderP = 0.005, kLeaderI = 0.0, kLeaderD = 0.0;
//     private double kLeaderFF = 0.0005;
//     private double kLeaderOutputMin = -1.0;
//     private double kLeaderOutputMax = 1.0;
//     private double kLeaderMaxRPM = 250;
//     private double kLeaderMaxAccel = 250;
    

//     public RotateSubsystem() {
//         rotateMotor = new SparkFlex(Constants.ArmConstants.ARM_MOTOR_PORT, MotorType.kBrushless);
//         SparkFlexConfig leaderConfig = new SparkFlexConfig();
//         AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
//         SoftLimitConfig leaderSoftLimit = new SoftLimitConfig();

//         rotatePIDController = rotateMotor.getClosedLoopController();

//         rotateEncoder = rotateMotor.getAbsoluteEncoder();

//         encoderConfig
//             .positionConversionFactor(360);
        
//         leaderConfig
//             .inverted(true)
//             .voltageCompensation(12.0)
//             .smartCurrentLimit(20)
//             .apply(encoderConfig)
//             .idleMode(IdleMode.kBrake)
//             .closedLoop
//                 .pidf(kLeaderP, kLeaderI, kLeaderD, kLeaderFF)
//                 .outputRange(kLeaderOutputMin, kLeaderOutputMax)
//                 .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//                 .maxMotion
//                     .maxAcceleration(kLeaderMaxAccel)
//                     .maxVelocity(kLeaderMaxRPM);
//         rotateMotor.configure(leaderConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


//         //TODO: Add soft limits
//         leaderSoftLimit
//         .forwardSoftLimit(60.0) 
//         .reverseSoftLimit(120.0)
//         .apply(leaderSoftLimit);
            
//         // Add motors to the simulation
//         if (Robot.isSimulation()) {
//             rotateMotorSim = new SparkFlexSim(rotateMotor, DCMotor.getNEO(1));
//             rotateEncoderSim = new SparkAbsoluteEncoderSim(rotateMotor);
//             rotateMotorSim.setPosition(90);
//             rotateEncoderSim.setPosition(90);
//             rotateMotorSim.setVelocity(0);
//             rotateEncoderSim.setVelocity(0);
//         }
//     }
    
//     // // An accessor method to set the speed (technically the output percentage) of the launch wheel
//     public void setArm(double pos) {
//         rotatePIDController.setReference(pos, SparkFlex.ControlType.kMAXMotionPositionControl);
//         if (Robot.isSimulation()) {
//             rotateMotorSim.setPosition(pos);
//             rotateEncoderSim.setPosition(pos);
//         }


//     }
    
//     public Command ForwardCmd() {
//     return this.runOnce(
//         () -> {
//             setArm(Constants.ArmConstants.ARM_OUT_POSE);
//         });
//     }

//     public Command MiddleCmd() {
//     return this.runOnce(
//         () -> {
//             setArm(Constants.ArmConstants.ARM_MIDDLE_POSE);
//         });
//     }

//     public Command UpCmd() {
//       return this.runOnce(
//           () -> {
//               setArm(Constants.ArmConstants.ARM_UP_POSE);
//           });
//       }
    
//     @Override
//     public void simulationPeriodic() {
//         // This method will be called once per scheduler run during simulation
//         if (Robot.isSimulation()) {
//             // rotateEncoderSim.setPosition(rotateMotorSim.getPosition());
//             // rotateMotorSim.iterate(rotateEncoderSim.getPosition(), rotateMotorSim.getBusVoltage(),.005);
//         }
//     }
    
//     @Override
//     public void periodic() {
//     // This method will be called once per scheduler run
//     if (Robot.isSimulation()) {
//         SmartDashboard.putNumber("Rotate Speed (RPM)", rotateEncoderSim.getPosition());
//     } else {
//         SmartDashboard.putNumber("Rotate Speed (RPM)", rotateEncoder.getPosition());
//     }
//     }
// }