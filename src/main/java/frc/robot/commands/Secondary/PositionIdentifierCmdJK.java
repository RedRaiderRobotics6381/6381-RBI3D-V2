// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

// import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.RotateSubsystem;

public class PositionIdentifierCmdJK extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final RotateSubsystem rotateSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier  oX, oY;
    // private final BooleanSupplier algeaBol;
    // private double pose; // initialize the pose variable
    // private double rotatePose; // initialize the rotatePose variable
    /**
     * Command to set the position of the elevator and rotate subsystems based on inputs from multiple buttons and a stick.
     *
     * @param elevatorSubsystem The subsystem responsible for controlling the elevator mechanism.
     * @param rotateSubsystem The subsystem responsible for controlling the rotation mechanism.
     * @param oX A DoubleSupplier providing the X coordinate of the input stick which will be rounded to 45 degree increments.
     * @param oY A DoubleSupplier providing the Y coordinate of the input stick which will be rounded to 45 degree increments.
     * @param algeaBol A BooleanSupplier indicating whether the button calling to pick up algea is pressed
     */
    public PositionIdentifierCmdJK(ElevatorSubsystem elevatorSubsystem, RotateSubsystem rotateSubsystem, IntakeSubsystem intakeSubsystem, DoubleSupplier oX, DoubleSupplier oY){
        
        this.elevatorSubsystem = elevatorSubsystem;
        this.rotateSubsystem = rotateSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.oX = oX;
        this.oY = oY;
        // this.algeaBol = algeaBol;

        addRequirements(elevatorSubsystem, rotateSubsystem, intakeSubsystem);  
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    /**
     * Executes the command to set the elevator and arm positions based on joystick input.
     * 
     * The method reads the joystick X and Y axis values to determine the input angle.
     * If the joystick is pushed, the input angle is snapped to the nearest 45-degree increment.
     * Based on the snapped input angle and the state of the algeaBol button, the method sets the 
     * elevator and arm positions to predefined constants.
     * 
     * The possible positions are:
     * - ALGAE_PICKUP_HIGH_POSE and ALGAE_INTAKE_POS if the joystick is pushed up and algeaBol is true.
     * - ALGAE_PICKUP_LOW_POSE and ALGAE_INTAKE_POS if the joystick is in the middle and algeaBol is true.
     * - REEF_HIGH_POSE and CORAL_HIGH_POS if the joystick is pushed up.
     * - REEF_MIDDLE_POSE and CORAL_HIGH_POS if the joystick is in the middle.
     * - REEF_LOW_POSE and CORAL_HIGH_POS if the joystick is pushed down.
     * 
     * Finally, the method sets the elevator height and arm position using the calculated pose values.
     */
    @Override
    public void execute() {
        // boolean inputAngleBol = false; // flag to track if the joystick is pushed
        double snappedInputAngle = 0; // initialize snappedInputAngle variable
        double oXRaw = oX.getAsDouble(); // get the joystick X axis values
        double oYRaw = oY.getAsDouble(); // get the joystick Y axis values
    
        // if (Math.abs(oXRaw) > 0.1 || Math.abs(oYRaw) > 0.1) {
        // if(Math.sqrt(Math.pow(oXRaw,2) + Math.pow(oYRaw, 2)) > 0.1) {
            // inputAngleBol = true; // if the joystick is pushed
            double inputAngle = Math.toDegrees(Math.atan2(oYRaw, oXRaw)) - 270; // -270 to make 0 degrees straight up
            inputAngle = (inputAngle + 360) % 360; // 360 degrees in a circle
            snappedInputAngle = Math.round(inputAngle / 45) * 45.0; // 45 degree increments
            snappedInputAngle = (snappedInputAngle + 360) % 360; // normalize to 0-360
            // System.out.println("Snapped Angle: " + snappedInputAngle + " oXRaw " + oXRaw + " oYRaw " + oYRaw);
        // }
        // System.out.println("Snapped Angle: " + snappedInputAngle);
        //Indetifier is the joystick value, 1 is up, -1 is down, 0 is middle
    
        if (snappedInputAngle == 315.0) { //if the joystick is pushed up and to the left
            Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.ALGAE_INTAKE_POS), rotateSubsystem)
                .alongWith(Commands.run(() -> intakeSubsystem.RunIntakeCmd(), intakeSubsystem))
                .andThen(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.ALGAE_PICKUP_HIGH_POSE), elevatorSubsystem)
                .andThen(() -> intakeSubsystem.HoldIntakeCmd(), intakeSubsystem)
                .andThen(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_MID_POS), rotateSubsystem)
                .schedule();
            // pose = Constants.ElevatorConstants.ALGAE_PICKUP_HIGH_POSE;
            // rotatePose = Constants.ArmConstants.ALGAE_INTAKE_POS;
        } else if (snappedInputAngle == 270.0) { //if the joystick is pushed left
            Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.ALGAE_INTAKE_POS), rotateSubsystem)
            .alongWith(Commands.run(() -> intakeSubsystem.RunIntakeCmd(), intakeSubsystem))
            .andThen(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.ALGAE_PICKUP_LOW_POSE), elevatorSubsystem)
            .andThen(() -> intakeSubsystem.HoldIntakeCmd(), intakeSubsystem)
            .andThen(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_MID_POS), rotateSubsystem)
            .schedule();
            // pose = Constants.ElevatorConstants.ALGAE_PICKUP_LOW_POSE;
            // rotatePose = Constants.ArmConstants.ALGAE_INTAKE_POS;
        } else if (snappedInputAngle == 45.0) { //if the joystick is pushed up and to the right
            Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_HIGH_POSE), elevatorSubsystem)
            .andThen(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_HIGH_POS), rotateSubsystem)
            .andThen(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_HIGH_POSE + 0.5), elevatorSubsystem)
            .andThen(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_INTAKE_POS), rotateSubsystem)
            .andThen(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.HUMAN_PLAYER_POSE), elevatorSubsystem)
            .schedule();
            // pose = Constants.ElevatorConstants.REEF_HIGH_POSE;
            // rotatePose = Constants.ArmConstants.CORAL_HIGH_POS;

        } else if (snappedInputAngle == 90.0) { //if the joystick is in the middle
            Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_MIDDLE_POSE), elevatorSubsystem)
            .andThen(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_MID_POS), rotateSubsystem)
            .andThen(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_MIDDLE_POSE + 0.5), elevatorSubsystem)
            .andThen(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_INTAKE_POS), rotateSubsystem)
            .andThen(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.HUMAN_PLAYER_POSE), elevatorSubsystem)
            .schedule();
            // pose = Constants.ElevatorConstants.REEF_MIDDLE_POSE;
            // rotatePose = Constants.ArmConstants.CORAL_MID_POS;
        } else if (snappedInputAngle == 135.0) { //if the joystick is pushed down
            Commands.run(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_LOW_POSE), elevatorSubsystem)
            .andThen(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_MID_POS), rotateSubsystem)
            .andThen(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_LOW_POSE + 0.5), elevatorSubsystem)
            .andThen(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_INTAKE_POS), rotateSubsystem)
            .andThen(() -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.HUMAN_PLAYER_POSE), elevatorSubsystem)
            .schedule();
            // pose = Constants.ElevatorConstants.REEF_LOW_POSE;
            // rotatePose = Constants.ArmConstants.CORAL_MID_POS;
        }
    

        // elevatorSubsystem.setElevatorHeight(pose);
        // if(Math.abs(pose - elevatorSubsystem.elevEncLdr.getPosition()) <= 0.125){
        //     rotateSubsystem.setArm(rotatePose);
        // }
    }
}
