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
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pid.TuneSmartMotionControl;
import frc.lib.pid.TuneVelocitySparkPIDController;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final SparkLimitSwitch downLimitSwitch = pivotIntakeMotor.getReverseLimitSwitch(Type.kNormallyOpen);
  private final SparkLimitSwitch upLimitSwitch = pivotIntakeMotor.getForwardLimitSwitch(Type.kNormallyClosed);

  /** Creates a new Intake. */
  public Intake() {
    intakeEncoderConfiguration = new CANcoderConfiguration();
    intakeEncoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    intakeEncoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    intakeEncoderConfiguration.MagnetSensor.MagnetOffset = Preferences.getDouble("intakeRotationOffset", 0.0) / 360.0;
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

  public void passGamePiece(double speedMetersPerSecond) {
    double speedInRPM = speedMetersPerSecond / (Math.PI * Constants.Intake.intakeWheelDiameter) * 60.0
        * Constants.Intake.intakeGearRatio;

    intakeController.setReference(speedInRPM, ControlType.kVelocity);
  }

  public void angleIntakeDown() {
    setElevation(Rotation2d.fromDegrees(Constants.Intake.downPositionDegrees));
  }

  public double getIntakeSpeed() {
    return (Constants.Intake.upperDistancePerMotorRotation * intakeMotor.getEncoder().getVelocity()) * 60;
  }

  public void angleIntakeBack() {
    setElevation(Rotation2d.fromDegrees(Constants.Intake.upPositionDegrees));
  }

  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
    intakeController.setReference(0, ControlType.kVelocity);
  }

  public void runIntakeElevationMotor(double percentVoltage) {
    pivotIntakeMotor.set(percentVoltage);
  }

  public void runIntakeMotor(double percentVoltage) {
    intakeMotor.set(percentVoltage);
  }

  public boolean hasNote() {
    return intakeBeamBreakSensor.isTriggered();
  }

  public boolean isBack() {
    return Math.abs(getAbsoluteRotationDegrees()
        - Constants.Intake.upPositionDegrees) < Constants.Intake.allowedAngleErrorInDegrees;
  }

  public boolean isDown() {
    return Math.abs(getAbsoluteRotationDegrees())
        - Constants.Intake.upPositionDegrees < Constants.Intake.allowedAngleErrorInDegrees;
  }

  public void setElevation(Rotation2d elevation) {
    double angleOfElevation = elevation.getDegrees() / Constants.Intake.ROTATION_DEGREES_PER_ROTATION;
    pivotController.setReference(angleOfElevation, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updateRotationOffset() {
    double currentOffset = intakeEncoderConfiguration.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - rotationAbsoluteSignal.getValue()) % 1.0;
    Preferences.setDouble("intakeRotationOffset", offset * 360.0);
    intakeEncoderConfiguration.MagnetSensor.MagnetOffset = offset;
    intakeAngleSensor.getConfigurator().apply(intakeEncoderConfiguration);
    syncRotationEncoders();
  }

  /**
   * Sync the relative rotation encoder (falcon) to the value of the absolute
   * encoder (CANCoder)
   */
  public void syncRotationEncoders() {
    intakeMotor.getEncoder().setPosition(getAbsoluteRotationDegrees() / Constants.Intake.ROTATION_DEGREES_PER_ROTATION);
  }

  public double getAbsoluteRotationDegrees() {
    return rotationAbsoluteSignal.getValueAsDouble() * 360;
  }

  public boolean onDownLimitSwitch() {
    return downLimitSwitch.isPressed();
  }

  public boolean onUpLimitSwitch() {
    return upLimitSwitch.isPressed();
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          setElevation(Rotation2d.fromDegrees(45));
        }, this),
        Commands.waitSeconds(1.0),
        Commands.runOnce(
            () -> {
              if (Math.abs(getAbsoluteRotationDegrees() - 45) > 2) {
                addFault(
                    "[System Check] Intake angle not in expected range :(",
                    false,
                    true);
              }

            }, this),
        Commands.runOnce(() -> {
          angleIntakeDown();
        }, this),
        Commands.waitSeconds(1.0),
        Commands.runOnce(
            () -> {
              if (!isDown()) {
                addFault(
                    "[System Check] The intake is not angled in the correct range for the downward position :(",
                    false,
                    true);
              }
            }, this),
        Commands.runOnce(() -> {
          runIntakeMotor(3);
        }, this),
        Commands.waitSeconds(1.0),
        Commands.runOnce(
            () -> {
              if (Math.abs(getIntakeSpeed() - 3) < 0.01) {
                addFault(
                    "[System Check] The Intake motors were unable to spin at the correct speed :(",
                    false,
                    true);
              }
              stopIntakeMotor();
            }, this),
        Commands.runOnce(() -> {
          angleIntakeBack();
        }, this),
        Commands.waitSeconds(2.0),
        Commands.runOnce(
          () -> {
              if (!isBack()) {
                addFault(
                    "[System Check] The intake is not angled in the correct range for the backward position :(",
                    false,
                    true);
              }}, this)
            
        
            )
        .until(() -> getFaults().size() > 0);

  }

  public Command getIntakeTuner() {
    return new TuneVelocitySparkPIDController("Intake", intakeMotor, this);
  }

  public Command getIntakePivotTuner() {
    return new TuneSmartMotionControl("Intake Pivot", pivotIntakeMotor, this);
  }
}
