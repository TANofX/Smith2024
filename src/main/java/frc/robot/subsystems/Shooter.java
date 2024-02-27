// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pid.TuneVelocitySparkPIDController;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NoteSensor;

public class Shooter extends AdvancedSubsystem {
  // private static final int Rotation2d = 0;
  private final CANSparkFlex topMotor = new CANSparkFlex(Constants.Shooter.topCANID, MotorType.kBrushless);
  private final CANSparkMax shooterIntakeMotor = new CANSparkMax(Constants.Shooter.intakeCANID, MotorType.kBrushless);
  private final SparkPIDController topController = topMotor.getPIDController();
  public final SparkPIDController shooterIntakeController = shooterIntakeMotor.getPIDController();
  private final NoteSensor shooterBeamBreakSensor = new NoteSensor(Constants.Shooter.noteSensorChannel);
  private double speedInRPM;

  /** Creates a new Shooter. */
  public Shooter() {
    registerHardware("Top Motor", topMotor);

    registerHardware("Shooters Intake Motor", shooterIntakeMotor);
    topMotor.getEncoder().getVelocity();
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
    speedInRPM = -1 * speedInMps / (Math.PI * Constants.Shooter.wheelDiameter) * 60.0
        * Constants.Shooter.gearRatioShooterSide;
    topController.setReference(speedInRPM, ControlType.kVelocity, 0);
  }

  public void stopMotors() {
    topMotor.set(0);
    shooterIntakeMotor.set(0);
  }

  public void intakeAtSpeed(double metersPerSecond) {
    double setPoint = (metersPerSecond / Constants.Shooter.intakeDistancePerMotorRotation) * 60.0;
    shooterIntakeController.setReference(setPoint, ControlType.kVelocity, 0);
  }

  public boolean hasNote() {
    return shooterBeamBreakSensor.isTriggered();
  }

  public double getShooterSpeed() {
    return topMotor.getEncoder().getVelocity();
  }

  // Uses encoder on motor to get the speed
  public double getTargetShooterSpeed() {
    return speedInRPM;
  }

  public boolean atSpeed() {
    double error = Math.abs(getTargetShooterSpeed() - getShooterSpeed());
    return error <= Math.abs(getTargetShooterSpeed() * 0.05);

  }

  public boolean readyToShoot() {
    return RobotContainer.shooterWrist.isAtElevation() && atSpeed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Ready to Shoot", readyToShoot());
    SmartDashboard.putBoolean("Is at Speed", atSpeed());
    SmartDashboard.putNumber("Target Shooter Speed", getTargetShooterSpeed());
    SmartDashboard.putNumber("Actual Shooter Speed", getShooterSpeed());
  }

  @Override
  protected Command systemCheckCommand() {

    // throw new UnsupportedOperationException("Unimplemented method
    // 'systemCheckCommand'");
    return Commands.runOnce(() -> {
    }, this);
  }

  public Command getIntakeTunerCommand() {
    return new TuneVelocitySparkPIDController("Shooter Intake", shooterIntakeMotor, this);
  }

  public Command getShooterTunerCommand() {
    return new TuneVelocitySparkPIDController("Shooter", topMotor, this);
  }
}
