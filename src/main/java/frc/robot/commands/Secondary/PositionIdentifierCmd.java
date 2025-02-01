// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.RotateSubsystem;




public class PositionIdentifierCmd extends Command {

  private final ElevatorSubsystem elevatorSubsystem;
  private final RotateSubsystem rotateSubsystem;
  private final DoubleSupplier  leftStick;
  private final BooleanSupplier leftBumper;


    public PositionIdentifierCmd(ElevatorSubsystem elevatorSubsystem, RotateSubsystem rotateSubsystem, DoubleSupplier leftStick, BooleanSupplier leftBumper)
    {
    this.elevatorSubsystem = elevatorSubsystem;
    this.rotateSubsystem = rotateSubsystem;
    this.leftStick = leftStick;
    this.leftBumper = leftBumper;


    addRequirements(elevatorSubsystem, rotateSubsystem);
    
  }

// Called when the command is initially scheduled.
@Override
public void initialize() {

}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {

double identifier = leftStick.getAsDouble();
double pose = 0;
double rotatePose = 0;

  
if (identifier > 0.33 && leftBumper.getAsBoolean()){
    pose = Constants.ElevatorConstants.ALGAE_PICKUP_HIGH_POSE;
    rotatePose = Constants.ArmConstants.ALGAE_INTAKE_POS;
  }

else if (identifier < 0.33 && identifier > 0.33 && leftBumper.getAsBoolean()){
    pose = Constants.ElevatorConstants.ALGAE_PICKUP_LOW_POSE;
    rotatePose = Constants.ArmConstants.ALGAE_INTAKE_POS;
  }  

// else if (identifier < -0.33 && leftBumper.getAsBoolean()){
//     pose = Constants.ElevatorConstants.TROUGH_POSE;
//   }

else if (identifier > 0.33){
    pose = Constants.ElevatorConstants.REEF_HIGH_POSE;
    rotatePose = Constants.ArmConstants.CORAL_HIGH_POS;
  }

else if (identifier < 0.33 && identifier > -0.33){
    pose = Constants.ElevatorConstants.REEF_MIDDLE_POSE;
    rotatePose = Constants.ArmConstants.CORAL_HIGH_POS;
  }
  
else if (identifier < -0.33){
    pose = Constants.ElevatorConstants.REEF_LOW_POSE;
    rotatePose = Constants.ArmConstants.CORAL_HIGH_POS;
  }

elevatorSubsystem.setElevatorHeight(pose);
rotateSubsystem.setArm(rotatePose);


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
