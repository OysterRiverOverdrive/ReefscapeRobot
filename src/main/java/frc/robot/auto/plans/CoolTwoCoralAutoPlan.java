package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.auto.AutoSleepCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;

public class CoolTwoCoralAutoPlan extends ParallelCommandGroup {

  public CoolTwoCoralAutoPlan(DrivetrainSubsystem drivetrain) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands

    Command showyDrive1 =
        autodrive.AutoDriveCmd(drivetrain, List.of(new Pose2d(-0.1, 0, new Rotation2d(0))));
    Command showyDrive2 =
        autodrive.AutoDriveCmd(drivetrain, List.of(new Pose2d(-3, -1, new Rotation2d(Math.PI))));
    addCommands(

        // Driving groups

        andThen(showyDrive1).andThen(new AutoSleepCmd(3)).andThen(showyDrive2));
  }
}
