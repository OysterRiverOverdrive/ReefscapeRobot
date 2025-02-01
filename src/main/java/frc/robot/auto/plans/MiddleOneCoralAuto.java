package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.auto.AutoFeederCmd;
import frc.robot.auto.AutoIntakeCmd;
import frc.robot.auto.AutoShooterCmd;
import frc.robot.auto.AutoSleepCmd;
import frc.robot.subsystems.DashboardSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.List;

public class MiddleOneCoralAuto extends ParallelCommandGroup {

  public MiddleOneCoralAuto(
        DrivetrainSubsystem drivetrain
        AlgaeArmSubsystem algaearm
        DashboardSubsystem dash
      //ElevatorSubsystem elevator
      ) {
    AutoCreationCmd autodrive = new AutoCreationCmd();

    //Auto driving commands

    Command 

    /*

    declare driving schematics

    AutoCreationCmd autodrive = new AutoCreationCmd();

    Ex.
    Command MoveToReef =
    autodrive.AutoDriveCmd(drivetrain, list.of(
    new Pose2d(325.68, 158.515, new Rotation2d(0)),
    new Pose2d(209.49, 158.515, new Rotation2d(0))
    ));

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


    Ex.
    andThen(alongWith(MoveToReef, New AutoElevatorCmd(elevator, ElevatorToLv1)))
    .andThen(New AutoElevatorCmd(elevator, ReleaseCoral))

    **this example code just drives straight to the reef from the middle starting point and unloads a coral onto the reef.

    */

  }
}
