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
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pid.TuneSmartMotionControl;
import frc.lib.pid.TuneSparkPIDController;
import frc.lib.pid.TuneVelocitySparkPIDController;
//import elevationMotor.getPIDController;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.util.NoteSensor;

public class Shooter extends AdvancedSubsystem {
  private static final int Rotation2d = 0;
  private final CANSparkFlex topMotor = new CANSparkFlex(Constants.Shooter.topCANID, MotorType.kBrushless);
  private final CANSparkMax shooterIntakeMotor = new CANSparkMax(Constants.Shooter.intakeCANID, MotorType.kBrushless);
  private final SparkPIDController topController = topMotor.getPIDController();
  public final SparkPIDController shooterIntakeController = shooterIntakeMotor.getPIDController();
  private final CANSparkFlex elevationMotor = new CANSparkFlex(Constants.Shooter.elevationCANID, MotorType.kBrushless);
  private final SparkPIDController elevationController = elevationMotor.getPIDController();
  public final CANcoder elevationEncoder = new CANcoder(Constants.Shooter.elevationEncoderCANID);
  private final CANcoderConfiguration shooterEncoderConfiguration;
  private final StatusSignal<Double> rotationAbsoluteSignal;
  private final NoteSensor shooterBeamBreakSensor = new NoteSensor(Constants.Shooter.noteSensorChannel);
  private double speedInRPM;
  private double targetElevation;


  /** Creates a new Shooter. */
  public Shooter() {

    registerHardware("Top Motor", topMotor);
    
    registerHardware("Shooters Intake Motor", shooterIntakeMotor);
    elevationMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    elevationMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    elevationMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(180.0 / Constants.Shooter.ROTATION_DEGREES_PER_ROTATION));
    elevationMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(-65.0 / Constants.Shooter.ROTATION_DEGREES_PER_ROTATION));
    registerHardware("Elevation Motor", elevationMotor);
    registerHardware("Elevation Encoder", elevationEncoder);
    shooterEncoderConfiguration = new CANcoderConfiguration();
    shooterEncoderConfiguration.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    shooterEncoderConfiguration.MagnetSensor.SensorDirection =
        SensorDirectionValue.Clockwise_Positive;
    shooterEncoderConfiguration.MagnetSensor.MagnetOffset =
        Preferences.getDouble("intakeRotationOffset", 0.0) / 360.0;
    elevationEncoder.getConfigurator().apply(shooterEncoderConfiguration);
    rotationAbsoluteSignal = elevationEncoder.getAbsolutePosition();
    rotationAbsoluteSignal.refresh();
    syncRotationEncoders();
    topMotor.getEncoder().getVelocity();
    elevationController.setP(Constants.Shooter.elevationMotorP, 0);
    elevationController.setI(Constants.Shooter.elevationMotorI, 0);
    elevationController.setD(Constants.Shooter.elevationMotorD, 0);
    elevationController.setFF(Constants.Shooter.elevationMotorFeedForward, 0);
    elevationController.setSmartMotionMaxAccel(Constants.Shooter.elevationMaxAcceleration, 0);
    elevationController.setSmartMotionMaxVelocity(Constants.Shooter.elevationMaxVelocity, 0);
    elevationController.setSmartMotionAllowedClosedLoopError(Constants.Shooter.elevationAllowedClosedLoopError, 0);
    shooterIntakeController.setP(Constants.Shooter.shooterIntakeMotorP, 0);
    shooterIntakeController.setI(Constants.Shooter.shooterIntakeMotorI, 0);
    shooterIntakeController.setD(Constants.Shooter.shooterIntakeMotorD, 0);
    shooterIntakeController.setFF(Constants.Shooter.shooterIntakeMotorFeedForward, 0);
    topController.setP(Constants.Shooter.shooterMotorP, 0);
    topController.setI(Constants.Shooter.shooterMotorI, 0);
    topController.setD(Constants.Shooter.shooterMotorD, 0);
    topController.setFF(Constants.Shooter.shooterMotorFeedForward, 0);    
    topController.setIZone(Constants.Shooter.shooterMotorIZone, 0);
  }
    
    public void stopIntakeMotor() {
      shooterIntakeMotor.set(0);
    }
    public void startMotorsForShooter(double speedInMps) {
      speedInRPM = -1*speedInMps/(Math.PI * Constants.Shooter.wheelDiameter)*60.0*Constants.Shooter.gearRatioShooterSide;
      topController.setReference(speedInRPM , ControlType.kVelocity, 0);
    }

    public void stopMotors() {
      topMotor.set(0);
      elevationMotor.set(0);
      shooterIntakeMotor.set(0);
    }
    public void intakeAtSpeed(double metersPerSecond) {
      double setPoint = (metersPerSecond / Constants.Shooter.intakeDistancePerMotorRotation) * 60.0;
      shooterIntakeController.setReference(setPoint, ControlType.kVelocity, 0);
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
    elevationMotor.getEncoder().setPosition(-1 * getAbsoluteRotationDegrees() / Constants.Shooter.ROTATION_DEGREES_PER_ROTATION);
  }
  public double getAbsoluteRotationDegrees() {
    return rotationAbsoluteSignal.getValue() * 360 * (-1.0); 
    //tells us what angle we are at
  }
  public void setElevation(Rotation2d elevation) {
    double angleOfElevation = (elevation.getDegrees() * -1.0) / Constants.Shooter.ROTATION_DEGREES_PER_ROTATION;
    targetElevation = elevation.getDegrees();
    elevationController.setReference(angleOfElevation,ControlType.kSmartMotion, 0);
  }
  public void raiseShooterManually() {
    elevationMotor.set(speedInRPM);
  }

  public boolean hasNote () {
    return shooterBeamBreakSensor.isTriggered();
  }
  public double getShooterSpeed () {
    return topMotor.getEncoder().getVelocity();
  }
  //Uses encoder on motor to get the speed
  public double getTargetShooterSpeed () {
    return speedInRPM;
  }
  public boolean atSpeed () {
    double error = Math.abs(getTargetShooterSpeed() - getShooterSpeed());
    return error <= Math.abs(getTargetShooterSpeed() * 0.05);
    
  }
  public boolean isAtElevation () {
    return Math.abs(getAbsoluteRotationDegrees() - targetElevation) <= Constants.Shooter.allowedErrorInDegreesForAngle;
  }
  public boolean readyToShoot () {
    return isAtElevation() && atSpeed();
  }
  
  /*public double angleToShootInAmp () {
    elevationMotor shootInAmpAngle(ControlType.kPosition);
  }*/

  @Override
  public void periodic() {
    rotationAbsoluteSignal.refresh();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Absolute Angle", rotationAbsoluteSignal.getValue() * 360);
    SmartDashboard.putNumber("Shooter Angle from Motor", elevationMotor.getEncoder().getPosition() * Constants.Shooter.ROTATION_DEGREES_PER_ROTATION);
    SmartDashboard.putNumber("Shooter Target Elevation", targetElevation);
    SmartDashboard.putBoolean("Ready to Shoot", readyToShoot());
    SmartDashboard.putBoolean("Is at Elevation", isAtElevation());
    SmartDashboard.putBoolean("Is at Speed", atSpeed());
    SmartDashboard.putNumber("Target Shooter Speed", getTargetShooterSpeed());
    SmartDashboard.putNumber("Actual Shooter Speed", getShooterSpeed());
  }

  @Override
  protected Command systemCheckCommand() {

    //throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
    return Commands.runOnce(() -> {}, this);
  }

  public Command getIntakeTunerCommand() {
    return new TuneVelocitySparkPIDController("Shooter Intake", shooterIntakeMotor, this);
  }

  public Command getShooterTunerCommand() {
    return new TuneVelocitySparkPIDController("Shooter", topMotor, this);
  }

  public Command getElevationTunerCommand() {
    return new TuneSmartMotionControl("Shooter Elevation", elevationMotor, this);
  }
  
}
