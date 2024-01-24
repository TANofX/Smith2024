// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;


public class Intake extends AdvancedSubsystem {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.intakeCANID, MotorType.kBrushless);
  private final CANSparkMax pivotIntakeMotor = new CANSparkMax(Constants.Intake.pivotIntakeCANID, MotorType.kBrushless);
  private final SparkPIDController pivotController = pivotIntakeMotor.getPIDController();
  private final SparkPIDController pivotIntakeController = pivotIntakeMotor.getPIDController();
  /** Creates a new Intake. */
  public Intake() {
    registerHardware("Intake Motor", intakeMotor);
    registerHardware("Pivot Intake Motor", pivotIntakeMotor);

  }
  public void intakeGamePiece() {
  pivotIntakeController.setReference(25, ControlType.kVelocity);
  }
  public void passGamePiece() {
    pivotIntakeController.setReference(100, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected Command systemCheckCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
  }
}
