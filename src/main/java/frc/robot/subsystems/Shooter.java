// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
//import elevationMotor.getPIDController;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.util.NoteSensor;

public class Shooter extends AdvancedSubsystem {
  private static final int Rotation2d = 0;
  private static final int ROTATION_DEGREES_PER_ROTATION = 0;
  private final CANSparkFlex topMotor = new CANSparkFlex(Constants.Shooter.topCANID, MotorType.kBrushless);
  private final CANSparkMax shooterIntakeMotor = new CANSparkMax(Constants.Shooter.intakeCANID, MotorType.kBrushless);
  private final SparkPIDController topController = topMotor.getPIDController();
  public final SparkPIDController shooterIntakeController = shooterIntakeMotor.getPIDController();
  private final CANSparkFlex elevationMotor = new CANSparkFlex(Constants.Shooter.elevationCANID, MotorType.kBrushless);
  private final SparkPIDController elevationController = elevationMotor.getPIDController();
  public final CANcoder elevationEncoder = new CANcoder(0);
  private final CANcoderConfiguration shooterEncoderConfiguration;
  private final StatusSignal<Double> rotationAbsoluteSignal;
  private final NoteSensor shooterBeamBreakSensor = new NoteSensor(Constants.Shooter.noteSensorChannel);
  private double speedInRPM;

  /** Creates a new Shooter. */
  public Shooter() {

    registerHardware("Top Motor", topMotor);
    
    registerHardware("Shooter Intake Motor", shooterIntakeMotor);
    registerHardware("Elevation Motor", elevationMotor);
    registerHardware("Elevation Encoder", elevationEncoder);
    shooterEncoderConfiguration = new CANcoderConfiguration();
    shooterEncoderConfiguration.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    shooterEncoderConfiguration.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    shooterEncoderConfiguration.MagnetSensor.MagnetOffset =
        Preferences.getDouble("intakeRotationOffset", 0.0) / 360.0;
    elevationEncoder.getConfigurator().apply(shooterEncoderConfiguration);
    rotationAbsoluteSignal = elevationEncoder.getAbsolutePosition();
    syncRotationEncoders();
    topMotor.getEncoder().getVelocity();
  }

    
    public void stopIntakeMotor() {
      shooterIntakeMotor.set(0);
    }
    public void shootGamePiece(double speedInMps) {
      speedInRPM = speedInMps/(Math.PI * Constants.Shooter.wheelDiameter)*60.0*Constants.Shooter.gearRatioShooterSide;
      topController.setReference(speedInRPM , ControlType.kVelocity);
    }

    public void stopMotors() {
      topMotor.set(0);
   
    }
    public void intakeAtSpeed(double metersPerSecond) {
      double setPoint = (metersPerSecond / Constants.Shooter.intakeDistancePerMotorRotation) * 60.0;
      shooterIntakeController.setReference(setPoint, ControlType.kVelocity);
    }
  public void updateRotationOffset() {
    double currentOffset = shooterEncoderConfiguration.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - rotationAbsoluteSignal.getValue()) % 1.0;
    Preferences.setDouble("intakeRotationOffset", offset * 360.0);
    shooterEncoderConfiguration.MagnetSensor.MagnetOffset = offset;
    elevationEncoder.getConfigurator().apply(shooterEncoderConfiguration);
    syncRotationEncoders();
  }

  /** Sync the relative rotation encoder (falcon) to the value of the absolute encoder (CANCoder) */
  public void syncRotationEncoders() {
    elevationMotor.getEncoder().setPosition(getAbsoluteRotationDegrees() / Constants.Shooter.ROTATION_DEGREES_PER_ROTATION);
  }
  public double getAbsoluteRotationDegrees() {
    return rotationAbsoluteSignal.getValueAsDouble() * 360; 
  }
  public void setElevation(Rotation2d elevation) {
    double angleOfElevation = elevation.getDegrees() / Constants.Shooter.ROTATION_DEGREES_PER_ROTATION;
    elevationController.setReference(angleOfElevation,ControlType.kPosition);
  }

  public boolean hasNote () {
    return shooterBeamBreakSensor.isTriggered();
  }
  public double getShooterSpeed () {
    return topMotor.getEncoder().getVelocity();
  }
  public double getTargetShooterSpeed () {
    return speedInRPM;
  }
  public boolean readyToShoot () {
    double error = Math.abs(getTargetShooterSpeed() - getShooterSpeed());
    return error <= Math.abs(getTargetShooterSpeed() * 0.01);
    
  }
  
  /*public double angleToShootInAmp () {
    elevationMotor shootInAmpAngle(ControlType.kPosition);
  }*/

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
