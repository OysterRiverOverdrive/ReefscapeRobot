package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.auto.AutoSleepCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;

public class AutoPlan extends ParallelCommandGroup {

  public AutoPlan(DrivetrainSubsystem drivetrain) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    // Auto Driving Commands

    Command showyDrive1 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Pose2d(.08, 0, new Rotation2d(0)), new Pose2d(.2, 0, new Rotation2d(0))));
    Command showyDrive2 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Pose2d(.85, 0, new Rotation2d(0)), new Pose2d(1.69, 0, new Rotation2d(0))));
    Command showyDrive3 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(
                new Pose2d(-.85, 0.01, new Rotation2d(0)),
                new Pose2d(-1.60, 0, new Rotation2d(0))));
    Command showyDrive4 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Pose2d(.5, 1, new Rotation2d(0)), new Pose2d(0.89, 2, new Rotation2d(0))));
    Command showyDrive5 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(
                new Pose2d(-.5, -1, new Rotation2d(0)),
                new Pose2d(-0.84, -1.44, new Rotation2d(0))));
    Command showyDrive6 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(new Pose2d(.5, -2, new Rotation2d(0)), new Pose2d(0.89, 0, new Rotation2d(0))));
    Command showyDrive7 =
        autodrive.AutoDriveCmd(
            drivetrain,
            List.of(
                new Pose2d(-.5, 1, new Rotation2d(0)), new Pose2d(-0.84, 0.5, new Rotation2d(0))));
    addCommands(

        // Driving groups
        new SequentialCommandGroup(
            showyDrive1,
            new AutoSleepCmd(1),
            showyDrive2,
            new AutoSleepCmd(0),
            showyDrive3,
            new AutoSleepCmd(.5),
            showyDrive4,
            new AutoSleepCmd(0),
            showyDrive5,
            new AutoSleepCmd(.5),
            showyDrive6,
            new AutoSleepCmd(0),
            showyDrive7));
  }
}
