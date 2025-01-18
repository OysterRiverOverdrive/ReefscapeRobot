package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.auto.AutoSleepCmd;

public class CircleTestAuto extends ParallelCommandGroup{
    
    public CircleTestAuto(
        DrivetrainSubsystem drivetrain
    ) {

    }

}
