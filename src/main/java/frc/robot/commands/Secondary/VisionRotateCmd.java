package frc.robot.commands.Secondary;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.RotateSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class VisionRotateCmd extends Command{
    private RotateSubsystem rotateSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    // private Vision vision;
    private double BargeXDist;
    private double elevatorHeight;
    private double robotHeight = 35.75;
    private double bargeHeight = 98;
    private double angle;
    public VisionRotateCmd(RotateSubsystem rotateSubsystem, SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem){
        this.rotateSubsystem = rotateSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        // vision = new Vision(this.swerveSubsystem::getPose, null);
    }
    @Override
    public void execute(){
        BargeXDist = 325.68 - swerveSubsystem.getPose().getX() * 39.3701;
        elevatorHeight = 3 * elevatorSubsystem.elevEncLdr.getPosition();
        angle = 240 - Math.toDegrees(Math.atan2(bargeHeight - robotHeight - elevatorHeight, BargeXDist));
        System.out.println("Angle: " + angle);

       // rotateSubsystem.setArm(angle);
    }
}