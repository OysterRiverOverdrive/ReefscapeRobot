// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class AlgaeSpinnerSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SparkMax Motor = new SparkMax(RobotConstants.kAlgaeSpinnerCanId, MotorType.kBrushless);

  public AlgaeSpinnerSubsystem() {}

  public void algaeSpinnerForwardCmd() {
    Motor.set(RobotConstants.kAlgaeSpinnerSpeed);
  }

  public void algaeSpinnerReverseCmd() {
    Motor.set(0 - RobotConstants.kAlgaeSpinnerSpeed);
  }

  public void algaeSpinnerStopCmd() {
    Motor.stopMotor();
  }
}
