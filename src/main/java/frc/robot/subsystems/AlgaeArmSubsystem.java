// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class AlgaeArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax m_AlgaeArmSparkMax =
      new SparkMax(RobotConstants.kAlgaeArmCanId, MotorType.kBrushless);

  private final RelativeEncoder m_AlgaeArmEncoder;

  private double elevatorSpeed;

  private final PIDController AlgaeArmPID =
      new PIDController(PIDConstants.kAlgaeArmP, PIDConstants.kAlgaeArmI, PIDConstants.kAlgaeArmD);

  public AlgaeArmSubsystem() {
    m_AlgaeArmEncoder = m_AlgaeArmSparkMax.getEncoder();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void toDown() {
    AlgaeArmPID.setSetpoint(RobotConstants.kAlgaeArmStops[0]);
  }

  public void toFlat() {
    AlgaeArmPID.setSetpoint(RobotConstants.kAlgaeArmStops[1]);
  }

  public void toRemoveAlgae() {
    AlgaeArmPID.setSetpoint(RobotConstants.kAlgaeArmStops[2]);
  }

  public void toUp() {
    AlgaeArmPID.setSetpoint(RobotConstants.kAlgaeArmStops[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorSpeed =
        AlgaeArmPID.calculate(m_AlgaeArmEncoder.getPosition() * RobotConstants.kAlgaeArmGearRatio);
    m_AlgaeArmSparkMax.set(elevatorSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
