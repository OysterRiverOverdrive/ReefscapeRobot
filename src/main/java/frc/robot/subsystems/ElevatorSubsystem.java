// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.RobotConstants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax m_elevator1SparkMax =
      new SparkMax(10, MotorType.kBrushless);

  private final SparkMax m_elevator2SparkMax =
      new SparkMax(11, MotorType.kBrushless);

  private final AbsoluteEncoder m_elevator1Encoder;

  private final SparkMaxConfig m_elevator2Config;

  private double elevatorSpeed;

  private final PIDController elevatorPID =
      new PIDController(PIDConstants.kElevatorP, PIDConstants.kElevatorI, PIDConstants.kElevatorD);

  public ElevatorSubsystem() {

    m_elevator2Config = new SparkMaxConfig();

    m_elevator1Encoder = m_elevator1SparkMax.getAbsoluteEncoder();

    m_elevator2Config.follow(m_elevator1SparkMax);

    m_elevator2SparkMax.configure(
        m_elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  public void runup() {
    m_elevator1SparkMax.set(0.3);
  }

  public void rundown() {
    m_elevator1SparkMax.set(-0.3);
  }

  public void stop() {
    m_elevator1SparkMax.stopMotor();
  }

  public void toBase() {
    elevatorPID.setSetpoint(ElevatorConstants.kElevatorStops[0]);
  }

  public void toL1() {
    elevatorPID.setSetpoint(ElevatorConstants.kElevatorStops[1]);
  }

  public void toL2() {
    elevatorPID.setSetpoint(ElevatorConstants.kElevatorStops[2]);
  }

  public void toL3() {
    elevatorPID.setSetpoint(ElevatorConstants.kElevatorStops[3]);
  }

  public void toL4() {
    elevatorPID.setSetpoint(ElevatorConstants.kElevatorStops[4]);
  }

  @Override
  public void periodic() {
    System.out.println(m_elevator1Encoder.getPosition());
    // This method will be called once per scheduler run
    // elevatorSpeed =
    //     elevatorPID.calculate(
    //         m_elevator1Encoder.getPosition() * ElevatorConstants.kElevatorHeightToRot);
    // m_elevator1SparkMax.set(elevatorSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
