// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.util.NoteSensor;


public class Intake extends AdvancedSubsystem {
  
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.intakeCANID, MotorType.kBrushless);
  private final CANSparkMax pivotIntakeMotor = new CANSparkMax(Constants.Intake.pivotIntakeCANID, MotorType.kBrushless);
  private final SparkPIDController pivotController = pivotIntakeMotor.getPIDController();
  private final SparkPIDController intakeController = intakeMotor.getPIDController();
  private final NoteSensor intakeBeamBreakSensor = new NoteSensor(Constants.Intake.intakeNoteSensorChannel);
  private final CANcoder intakeAngleSensor = new CANcoder(Constants.Intake.intakeAngleSensor);
  private final StatusSignal<Double> rotationAbsoluteSignal;
  private final CANcoderConfiguration intakeEncoderConfiguration;
  /** Creates a new Intake. */
  public Intake() {
     intakeEncoderConfiguration = new CANcoderConfiguration();
    intakeEncoderConfiguration.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    intakeEncoderConfiguration.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    intakeEncoderConfiguration.MagnetSensor.MagnetOffset =
        Preferences.getDouble("intakeRotationOffset", 0.0) / 360.0;
    intakeAngleSensor.getConfigurator().apply(intakeEncoderConfiguration);
    registerHardware("Intake Motor", intakeMotor);
    registerHardware("Pivot Intake Motor", pivotIntakeMotor);
    registerHardware("Intake Angle Sensor", intakeAngleSensor);
    rotationAbsoluteSignal = intakeAngleSensor.getAbsolutePosition();
    syncRotationEncoders();
    

  }
  public void intakeGamePiece() {
  intakeController.setReference(25, ControlType.kVelocity);
  }
  public void passGamePiece() {
    intakeController.setReference(100, ControlType.kVelocity);
  }
  public void angleIntakeDown() {
    pivotController.setReference(180, ControlType.kPosition);
  }
  public void angleIntakeBack() {
    pivotController.setReference(10, ControlType.kPosition);
  }
  public void stopIntakeMotor () {
intakeMotor.stopMotor();
intakeController.setReference(0, ControlType.kVelocity);
  }
 public boolean hasNote () {
  return intakeBeamBreakSensor.isTriggered();
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
  public void updateRotationOffset() {
    double currentOffset = intakeEncoderConfiguration.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - rotationAbsoluteSignal.getValue()) % 1.0;
    Preferences.setDouble("intakeRotationOffset", offset * 360.0);
    intakeEncoderConfiguration.MagnetSensor.MagnetOffset = offset;
    intakeAngleSensor.getConfigurator().apply(intakeEncoderConfiguration);
    syncRotationEncoders();
  }

  /** Sync the relative rotation encoder (falcon) to the value of the absolute encoder (CANCoder) */
  public void syncRotationEncoders() {
    intakeMotor.getEncoder().setPosition(getAbsoluteRotationDegrees() / Constants.Intake.ROTATION_DEGREES_PER_ROTATION);
  }
  public double getAbsoluteRotationDegrees() {
    return rotationAbsoluteSignal.getValueAsDouble() * 360; 
  }
}
