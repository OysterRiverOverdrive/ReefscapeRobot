package frc.robot.auto.plans;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoPlanEx extends ParallelCommandGroup {

  public AutoPlanEx(DrivetrainSubsystem drivetrain) {

    /*

    declare driving schematics

    AutoCreationCmd autodrive = new AutoCreationCmd();

    Ex.
    Command RightShoot =
    autodrive.AutoDriveCmd(
        drivetrain,
        List.of(new Translation2d(0.3, 0)),
        new Pose2d(
            0.76,
            dash.getAlliance() * 0.12,
            new Rotation2d(dash.getAlliance() * -2 * Math.PI / 3)));

    --Then for formatting event-based commands toegther--

    function andThen() is a sequential command group
        -commands one after another
    function alongWith() is a parallel command group
        -commands with each other
    function deadlineWith() runs commands alongside the previous group until one of the commands stop.
        -the first command to stop will stop the entire group

    first:
    andThen(alongWith(Command... commands that are going to run))
    .andThen(alongWith())
    ...
        .deadlineWith()
    and that's the first cycle :)


    */

  }
}
