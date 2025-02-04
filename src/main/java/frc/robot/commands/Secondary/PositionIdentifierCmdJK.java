// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.RotateSubsystem;

public class PositionIdentifierCmdJK extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final RotateSubsystem rotateSubsystem;
    private final DoubleSupplier  oX, oY;
    private final BooleanSupplier algeaBol;
    private double pose; // initialize the pose variable
    private double rotatePose; // initialize the rotatePose variable
    /**
     * Command to set the position of the elevator and rotate subsystems based on inputs from multiple buttons and a stick.
     *
     * @param elevatorSubsystem The subsystem responsible for controlling the elevator mechanism.
     * @param rotateSubsystem The subsystem responsible for controlling the rotation mechanism.
     * @param oX A DoubleSupplier providing the X coordinate of the input stick which will be rounded to 45 degree increments.
     * @param oY A DoubleSupplier providing the Y coordinate of the input stick which will be rounded to 45 degree increments.
     * @param algeaBol A BooleanSupplier indicating whether the button calling to pick up algea is pressed
     */
    public PositionIdentifierCmdJK(ElevatorSubsystem elevatorSubsystem, RotateSubsystem rotateSubsystem, DoubleSupplier oX, DoubleSupplier oY, BooleanSupplier algeaBol){
        
        this.elevatorSubsystem = elevatorSubsystem;
        this.rotateSubsystem = rotateSubsystem;
        this.oX = oX;
        this.oY = oY;
        this.algeaBol = algeaBol;

        addRequirements(elevatorSubsystem, rotateSubsystem);  
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
    // double identifier = oX.getAsDouble(); //    double identifier = leftStick.getAsDouble();

        boolean inputAngleBol = false; // flag to track if the joystick is pushed
        double snappedInputAngle = 0; // initialize snappedInputAngle variable
        double oXRaw = oX.getAsDouble(); // get the joystick X axis values
        double oYRaw = oY.getAsDouble(); // get the joystick Y axis values
    
        if (Math.abs(oXRaw) > 0.1 || Math.abs(oYRaw) > 0.1) {
            //if(Math.sqrt(Math.Pow(oxRaw,2) + Math.Pow(oYRaw, 2)) > 0.1)
            inputAngleBol = true; // if the joystick is pushed
            double inputAngle = Math.toDegrees(Math.atan2(oYRaw, oXRaw)) - 270; // -270 to make 0 degrees straight up
            inputAngle = (inputAngle + 360) % 360; // 360 degrees in a circle
            snappedInputAngle = Math.round(inputAngle / 45) * 45.0; // 45 degree increments
            snappedInputAngle = (snappedInputAngle + 360) % 360; // normalize to 0-360
            // System.out.println("Snapped Angle: " + snappedInputAngle + " oXRaw " + oXRaw + " oYRaw " + oYRaw);
        }
        // System.out.println("Snapped Angle: " + snappedInputAngle);
        //Indetifier is the joystick value, 1 is up, -1 is down, 0 is middle
    
        if (inputAngleBol && snappedInputAngle == 0.0 && algeaBol.getAsBoolean()) { //if the left bumper is pressed and the joystick is pushed up
                    // Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.ALGAE_INTAKE_POS), rotateSubsystem).andThen(
                    //      () -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.ALGAE_PICKUP_HIGH_POSE), elevatorSubsystem);
                    //       System.out.println("a");
            pose = Constants.ElevatorConstants.ALGAE_PICKUP_HIGH_POSE;
        rotatePose = Constants.ArmConstants.ALGAE_INTAKE_POS;
    // } else if (identifier <= 0.33 && identifier > -0.33 && leftBumper.getAsBoolean()) { //if the left bumper is pressed and the joystick is in the middle
    } else if (!inputAngleBol && algeaBol.getAsBoolean()) {
        // Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.ALGAE_INTAKE_POS), rotateSubsystem).andThen(
        //     () -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.ALGAE_PICKUP_LOW_POSE), elevatorSubsystem);
        //     System.out.println("B");
        pose = Constants.ElevatorConstants.ALGAE_PICKUP_LOW_POSE;
        rotatePose = Constants.ArmConstants.ALGAE_INTAKE_POS;
    } else if (inputAngleBol && snappedInputAngle == 0.0) { //if the joystick is pushed up
        pose = Constants.ElevatorConstants.REEF_HIGH_POSE;
        
        
        rotatePose = Constants.ArmConstants.CORAL_HIGH_POS;
        // Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_HIGH_POS), rotateSubsystem).andThen(
        //     () -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_HIGH_POSE), elevatorSubsystem);
        //     System.out.println("C");
    } else if (!inputAngleBol) { //if the joystick is in the middle
        // Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_HIGH_POS), rotateSubsystem).andThen(
        //              () -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_MIDDLE_POSE), elevatorSubsystem);
        //              System.out.println("d");
        pose = Constants.ElevatorConstants.REEF_MIDDLE_POSE;
        rotatePose = Constants.ArmConstants.CORAL_MID_POS;
    } else if (inputAngleBol && snappedInputAngle == 180.0) { //if the joystick is pushed down
        // Commands.run(() -> rotateSubsystem.setArm(Constants.ArmConstants.CORAL_HIGH_POS), rotateSubsystem).andThen(
        //     () -> elevatorSubsystem.setElevatorHeight(Constants.ElevatorConstants.REEF_LOW_POSE), elevatorSubsystem);
        //     System.out.println("e");
        pose = Constants.ElevatorConstants.REEF_LOW_POSE;
        rotatePose = Constants.ArmConstants.CORAL_MID_POS;
    }
    
    // if (identifier > 0.33 && leftBumper.getAsBoolean()){ //if the left bumper is pressed and the joystick is pushed up
    //     pose = Constants.ElevatorConstants.ALGAE_PICKUP_HIGH_POSE;
    //     rotatePose = Constants.ArmConstants.ALGAE_INTAKE_POS;
    //   }

    // else if (identifier < 0.33 && identifier > 0.33 && leftBumper.getAsBoolean()){ //if the left bumper is pressed and the joystick is in the middle
    //     pose = Constants.ElevatorConstants.ALGAE_PICKUP_LOW_POSE;
    //     rotatePose = Constants.ArmConstants.ALGAE_INTAKE_POS;
    //   }  

    // // else if (identifier < -0.33 && leftBumper.getAsBoolean()){
    // //     pose = Constants.ElevatorConstants.TROUGH_POSE;
    // //   }

    // else if (identifier > 0.33){ //if the joystick is pushed up
    //     pose = Constants.ElevatorConstants.REEF_HIGH_POSE;
    //     rotatePose = Constants.ArmConstants.CORAL_HIGH_POS;
    //   }

    // else if (identifier < 0.33 && identifier > -0.33){ //if the joystick is in the middle
    //     pose = Constants.ElevatorConstants.REEF_MIDDLE_POSE;
    //     rotatePose = Constants.ArmConstants.CORAL_HIGH_POS;
    //   }
    
    // else if (identifier < -0.33){ //if the joystick is pushed down
    //     pose = Constants.ElevatorConstants.REEF_LOW_POSE;
    //     rotatePose = Constants.ArmConstants.CORAL_HIGH_POS;
    //   }

    elevatorSubsystem.setElevatorHeight(pose);
    if(Math.abs(pose - elevatorSubsystem.elevEncLdr.getPosition()) <= 0.125){
        rotateSubsystem.setArm(rotatePose);
    }
    // Commands.run(() -> elevatorSubsystem.setElevatorHeight(pose)).andThen(
    //     () -> rotateSubsystem.setArm(rotatePose));
}


// // Called once the command ends or is interrupted.
// @Override
// public void end(boolean interrupted) {

// }

// // Returns true when the command should end.
// @Override
// public boolean isFinished() {

// }
}
