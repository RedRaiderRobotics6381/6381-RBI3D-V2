// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.proto.System;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.DrivebaseConstants;
// import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Secondary.ElevatorInitCmd;
// import frc.robot.commands.Secondary.PositionIdentifierCmd;
import frc.robot.commands.Secondary.PositionIdentifierCmd;
import frc.robot.commands.Secondary.VisionRotateCmd;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdvHdg;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.RotateSubsystem;
// import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
// import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdvAim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
// import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

import java.io.File;
// import swervelib.SwerveInputStream;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController engineerXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  public final RotateSubsystem rotateSubsystem = new RotateSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final VisionRotateCmd visionRotateCmd = new VisionRotateCmd(rotateSubsystem, drivebase, elevatorSubsystem);

  public double currentSnappedAngle = 0;
  public double snappedAngle = 0;       

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // rotation control is selectable between direct angle and angular velocity
  // left stick controls translation
  // in one mode the right stick controls the rotational velocity 
  // in the other mode the right stick controls the desired angle NOT angular rotation
  // also in this mode the POV buttons are used to quickly face a direction
  // and a button will yaw the robot towards a target.
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  Command AbsoluteDriveAdvHdg = new AbsoluteDriveAdvHdg(drivebase,
                                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                  OperatorConstants.LEFT_Y_DEADBAND) *
                                                                                                  DrivebaseConstants.Max_Speed_Multiplier,
                                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                  OperatorConstants.LEFT_X_DEADBAND) *
                                                                                                  DrivebaseConstants.Max_Speed_Multiplier,
                                                                    () -> -MathUtil.applyDeadband(driverXbox.getRightX(),OperatorConstants.LEFT_X_DEADBAND),
                                                                    () -> -MathUtil.applyDeadband(driverXbox.getRightY(),OperatorConstants.LEFT_Y_DEADBAND),
                                                                    () -> -MathUtil.applyDeadband(driverXbox.getLeftTriggerAxis(),OperatorConstants.LEFT_Y_DEADBAND),
                                                                    () -> -MathUtil.applyDeadband(driverXbox.getRightTriggerAxis(),OperatorConstants.LEFT_Y_DEADBAND),
                                                                    () -> driverXbox.getHID().getPOV(),
                                                                    driverXbox.rightStick());

  // /**
  //  * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
  //  */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  // /**
  //  * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
  //  */
  // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
  //                                                                                            driverXbox::getRightY)
  //                                                          .headingWhile(true);


  // // Applies deadbands and inverts controls because joysticks
  // // are back-right positive while robot
  // // controls are front-left positive
  // // left stick controls translation
  // // right stick controls the desired angle NOT angular rotation
  // Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // // Applies deadbands and inverts controls because joysticks
  // // are back-right positive while robot
  // // controls are front-left positive
  // // left stick controls translation
  // // right stick controls the angular velocity of the robot
  // Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  // SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //                                                                  () -> -driverXbox.getLeftY(),
  //                                                                  () -> -driverXbox.getLeftX())
  //                                                              .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
  //                                                              .deadband(OperatorConstants.DEADBAND)
  //                                                              .scaleTranslation(0.8)
  //                                                              .allianceRelativeControl(true);
  // // Derive the heading axis with math!
  // SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
  //                                                                    .withControllerHeadingAxis(() -> Math.sin(
  //                                                                                                   driverXbox.getRawAxis(
  //                                                                                                       2) * Math.PI) * (Math.PI * 2),
  //                                                                                               () -> Math.cos(
  //                                                                                                   driverXbox.getRawAxis(
  //                                                                                                       2) * Math.PI) *
  //                                                                                                     (Math.PI * 2))
  //                                                                    .headingWhile(true);

  // Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  // Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

  // Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("AlgaeIntakePos", rotateSubsystem.RotatePosCmd(Constants.ArmConstants.ALGAE_INTAKE_POS));
    NamedCommands.registerCommand("CoralHighOuttake", rotateSubsystem.RotatePosCmd(Constants.ArmConstants.CORAL_HIGH_POS));
    NamedCommands.registerCommand("ElevatorHigh", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_HIGH_POSE));
    NamedCommands.registerCommand("AlgaeIntake", intakeSubsystem.IntakeCmd());
    NamedCommands.registerCommand("AlgaeOuttake", intakeSubsystem.OuttakeCmd());
    NamedCommands.registerCommand("CoralIntakePos", rotateSubsystem.RotatePosCmd(Constants.ArmConstants.CORAL_INTAKE_POS));
    NamedCommands.registerCommand("AlgaeHold", intakeSubsystem.RunIntakeCmd(0.1));
    NamedCommands.registerCommand("CoralHoldPos", rotateSubsystem.RotatePosCmd(Constants.ArmConstants.ALGAE_BARGE_POS));
    NamedCommands.registerCommand("ElevatorDown", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.START_POSE));
    NamedCommands.registerCommand("CoralHighOuttake+0.5", rotateSubsystem.RotatePosCmd(Constants.ArmConstants.CORAL_HIGH_POS + 0.5));
    // NamedCommands.registerCommand("initElevator", ElevatorInitCmd);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // (Condition) ? Return-On-True : Return-on-False
    // drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
    //                             driveFieldOrientedAnglularVelocity :
    //                             driveFieldOrientedAnglularVelocitySim);
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                AbsoluteDriveAdvHdg :
                                AbsoluteDriveAdvHdg);

    if (Robot.isSimulation())
    {
      driverXbox.back().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      // driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    } else
    {
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());

      driverXbox.b().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
                          Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                                                        new Transform2d(0.6604, -.164338,
                                                        Rotation2d.fromDegrees(180))))));

      driverXbox.x().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
                          Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                                                        new Transform2d(0.6604, .164338,
                                                        Rotation2d.fromDegrees(180))))));

      driverXbox.y().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
                          Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                                                        new Transform2d(0.5, 0.0,
                                                        Rotation2d.fromDegrees(0.0))))));

      driverXbox.a().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
                          Vision.getAprilTagPose(AprilTagConstants.HumanPlayerRight,
                                                        new Transform2d(1.0, 0.0,
                                                        Rotation2d.fromDegrees(180.0))))));
                                                        //.alongWith(Commands.run(() -> SmartDashboard.putNumber("Offset", vision.getDistanceFromAprilTag(AprilTagConstants.HumanPlayerRight)))));


      // engineerXbox.leftStick().and(engineerXbox.x()).whileTrue(intakeSubsystem.IntakeCmd());
      // engineerXbox.leftStick().and(engineerXbox.y()).whileTrue(intakeSubsystem.OuttakeCmd());
     // engineerXbox.leftBumper().whileTrue(intakeSubsystem.IntakeCmd());
      engineerXbox.rightBumper().whileTrue(intakeSubsystem.OuttakeCmd());

      // engineerXbox.a().whileTrue(new PositionIdentifierCmd(elevatorSubsystem,
      //                                                  rotateSubsystem,
      //                                                  () -> engineerXbox.getLeftX(),
      //                                                  engineerXbox.leftBumper()));

      engineerXbox.leftStick().whileTrue(new PositionIdentifierCmd(elevatorSubsystem,
                                                                      rotateSubsystem,
                                                                      intakeSubsystem, 
                                                                      () -> engineerXbox.getLeftX(),
                                                                      () -> engineerXbox.getLeftY()));

      // engineerXbox.y().whileTrue(Commands.runEnd((() -> intakeSubsystem.runIntake(Constants.IntakeConstants.OUTTAKE_SPEED)),
      //                                          (() -> intakeSubsystem.runIntake(Constants.IntakeConstants.STOP_SPEED)),
      //                                                 intakeSubsystem));

      
      // engineerXbox.rightStick().and(engineerXbox.x()).onTrue(rotateSubsystem.ForwardCmd());
      // engineerXbox.rightStick().and(engineerXbox.y()).onTrue(rotateSubsystem.UpCmd());
      // engineerXbox.rightStick().and(engineerXbox.b()).onTrue(rotateSubsystem.MiddleCmd());
      // engineerXbox.rightStick().and(engineerXbox.a()).onTrue(rotateSubsystem.AlgaeIntakeCmd());

      // engineerXbox.leftStick().negate().and(engineerXbox.x()).onTrue(rotateSubsystem.ForwardCmd()); //Rotate arm to 190 degrees CORAL_INTAKE_POS
      // engineerXbox.leftStick().negate().and(engineerXbox.y()).onTrue(rotateSubsystem.UpCmd()); //Rotate arm to 165 degrees CORAL_HIGH_POS
      // engineerXbox.leftStick().negate().and(engineerXbox.b()).onTrue(rotateSubsystem.MiddleCmd()); //Rotate arm to 175 degrees CORAL_MID_POS
      // engineerXbox.leftStick().negate().and(engineerXbox.a()).onTrue(rotateSubsystem.AlgaeIntakeCmd()); //Rotate arm to 240 degrees ALGAE_INTAKE_POS
      // engineerXbox.pov(90).onTrue(rotateSubsystem.AlgaeStartCmd()); //Rotate arm to 280 degrees ALGAE_START_POS      

      engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.x())).onTrue(Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_INTAKE_POS), rotateSubsystem)); //Rotate arm to 190 degrees CORAL_INTAKE_POS
      engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.y())).onTrue(Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_HIGH_POS), rotateSubsystem)); //Rotate arm to 165 degrees CORAL_HIGH_POS
      engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.b())).onTrue(Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_MID_POS), rotateSubsystem)); //Rotate arm to 175 degrees CORAL_MID_POS
      engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.a())).onTrue(Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.ALGAE_INTAKE_POS), rotateSubsystem)); //Rotate arm to 240 degrees ALGAE_INTAKE_POS
      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.pov(90))).onTrue(Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.ALGAE_START_POS), rotateSubsystem)); //Rotate arm to 280 degrees ALGAE_START_POS

      // engineerXbox.x().onTrue(Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.ARM_OUT_POSE), rotateSubsystem));
      // engineerXbox.b().onTrue(Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.ARM_UP_POSE), rotateSubsystem));
      // engineerXbox.y().onTrue(Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.ARM_MIDDLE_POSE), rotateSubsystem));

      engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.a())).onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.START_POSE), elevatorSubsystem)); //Set elevator to 0
      engineerXbox.leftBumper().whileTrue(visionRotateCmd);
      // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.x())).onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_LOW_POSE), elevatorSubsystem)); //Set elevator to 4
      // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.y())).onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_MIDDLE_POSE), elevatorSubsystem)); //Set elevator to 6
      // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.b())).onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_HIGH_POSE), elevatorSubsystem)); //Set elevator to 14.75

      // engineerXbox.a().onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.TROUGH_POSE), elevatorSubsystem));
      // engineerXbox.x().onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_LOW_POSE), elevatorSubsystem));
      // engineerXbox.y().onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_MIDDLE_POSE), elevatorSubsystem));
      // engineerXbox.b().onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_HIGH_POSE), elevatorSubsystem));

      // engineerXbox.a().onTrue(elevatorSubsystem.START_POSE());
      // engineerXbox.x().onTrue(elevatorSubsystem.REEF_LOW_POSE());
      // engineerXbox.y().onTrue(elevatorSubsystem.REEF_MIDDLE_POSE());
      // engineerXbox.b().onTrue(elevatorSubsystem.REEF_HIGH_POSE());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Sussex_1Coral+1Algae");
  }

  public void setMotorBrake(boolean brake)
  {

    drivebase.setMotorBrake(brake);
  }
  public void initElevator(){
    new ElevatorInitCmd(elevatorSubsystem).schedule();
  }

  /**
   * Sets the speed multiplier for the drivebase based on the state of the right and left bumpers on the driver's Xbox controller.
   * If both bumpers are pressed, the speed multiplier is set to 1 (HighSpd).
   * If either bumper is pressed, the speed multiplier is set to 0.75 (MedSpd).
   * If neither bumper is pressed, the speed multiplier is set to 0.50 (LowSpd).
   */
  public void spencerButtons(){

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("HighSpd");
      DrivebaseConstants.Max_Speed_Multiplier = 1;
    }

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == false ||
        driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("MedSpd");
      DrivebaseConstants.Max_Speed_Multiplier = .875;
    }

    if (driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == false){
      //System.out.println("LowSpd");
      DrivebaseConstants.Max_Speed_Multiplier = .75;
    }
    
  }

    /**
   * Calculates the snapped angle based on the current heading of the swerve drive.
   * The angle is normalized to the range [0, 360) and then snapped to the nearest 60-degree increment.
   * If the snapped angle changes, it updates the current snapped angle and sets the appropriate AprilTag ID
   * based on the alliance color (Red or Blue).
   *
   * The AprilTag ID is determined as follows:
   * - For the Red alliance:
   *   - 0.0 degrees: AprilTag ID 7
   *   - 60.0 degrees: AprilTag ID 8
   *   - 120.0 degrees: AprilTag ID 9
   *   - 180.0 degrees: AprilTag ID 10
   *   - 240.0 degrees: AprilTag ID 11
   *   - 300.0 degrees: AprilTag ID 6
   * - For the Blue alliance:
   *   - 0.0 degrees: AprilTag ID 18
   *   - 60.0 degrees: AprilTag ID 19
   *   - 120.0 degrees: AprilTag ID 20
   *   - 180.0 degrees: AprilTag ID 21
   *   - 240.0 degrees: AprilTag ID 22
   *   - 300.0 degrees: AprilTag ID 17
   */
  void getSnappedAngle(){
    Rotation2d currentHeading = drivebase.getHeading();   
    double angle = Math.toDegrees(Math.atan2(currentHeading.getCos(), currentHeading.getSin()))-270;
    // Normalize to the range [0, 360)
    angle = (angle + 360) % 360;

    // Snap to the nearest 60-degree increment
    snappedAngle = Math.round(angle / 60.0) * 60.0;
    if (snappedAngle != currentSnappedAngle){
      currentSnappedAngle = snappedAngle;
      
      // if (headingX != 0 || headingY != 0) {
      //   headingX = Math.sin(Math.toRadians(snappedAngle));
      //   headingY = Math.cos(Math.toRadians(snappedAngle));

      Optional<Alliance> allianceColor = DriverStation.getAlliance();
      if (allianceColor.isPresent()) {
        if (allianceColor.get() == Alliance.Red) {
          if(snappedAngle == 180.0){AprilTagConstants.ReefTagID = 7 ;};
          if(snappedAngle == 240.0){AprilTagConstants.ReefTagID = 8 ;};
          if(snappedAngle == 300.0){AprilTagConstants.ReefTagID = 9 ;};
          if(snappedAngle == 0.0){AprilTagConstants.ReefTagID = 10;};
          if(snappedAngle == 60.0){AprilTagConstants.ReefTagID = 11;};
          if(snappedAngle == 120.0){AprilTagConstants.ReefTagID = 6 ;};
        }
        else if (allianceColor.get() == Alliance.Blue) {
          if(snappedAngle ==   180.0){AprilTagConstants.ReefTagID = 18;};
          if(snappedAngle ==   240.0){AprilTagConstants.ReefTagID = 19;};
          if(snappedAngle ==   300){AprilTagConstants.ReefTagID = 20;};
          if(snappedAngle ==   0.0){AprilTagConstants.ReefTagID = 21;};
          if(snappedAngle ==   60.0){AprilTagConstants.ReefTagID = 22;};
          if(snappedAngle ==   120.0){AprilTagConstants.ReefTagID = 17;};
        }
      }

    }
  }
    //Button 1 is "A" on xbox controller
    //Button 2 is "B" on xbox controller
    //Button 3 is "X" on xbox controller  
    //Button 4 is "Y" on xbox controller
    //Button 5 is "Left Bumper" on xbox controller
    //Button 6 is "Right Bumper" on xbox controller
    //Button 7 is "Back" on xbox controller
    //Button 8 is "Start" on xbox controller
    //Button 9 is "Left Joystick" on xbox controller
    //Button 10 is "Right Joystick" on xbox controller
    //Axis 0 is left joystick x side to side
    //Axis 1 is left joystick y forward and back
    //Axis 2 is left trigger 
    //Axis 3 is right trigger
    //Axis 4 is right joystick x side to side
    //Axis 5 is right joystick y forward and back
}
