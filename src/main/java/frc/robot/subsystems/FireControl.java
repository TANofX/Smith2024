// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class FireControl extends SubsystemBase {
  public enum TargetLocation {
    AMP,
    SPEAKER,
    NONE
  }
  private Supplier<Pose2d> poseSupplier;
  private Supplier<Optional<Alliance>> allianceSupplier;
  private boolean trackTarget = false;
  private TargetLocation targetingMode = TargetLocation.NONE;
  private final PIDController speakerController;
  private double rot;

  /** Creates a new FireControl. */
  public FireControl(Supplier<Pose2d> poseSupplier, Supplier<Optional<Alliance>> allianceSupplier) {
    this.poseSupplier = poseSupplier;
    this.allianceSupplier = allianceSupplier;
    this.speakerController = new PIDController(2.0, 0, 0.025);//new PIDController(1.5, 1.5, 0.025); //MAKE CONSTANTS
    speakerController.setIntegratorRange(-0.5, 0.5);
    speakerController.setIZone(15);
    speakerController.setTolerance(1.75 * Math.PI / 180.0);
  }

  @Override
  public void periodic() {
    Pose2d currentPosition = poseSupplier.get();
    double distanceFromSpeaker;

    // Rotation2d robotDesiredAngle;
    Optional<Alliance> currentAlliance = allianceSupplier.get();

    Pose2d shooterPose = currentPosition; // .plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)));
    // Transform2d shooterToSpeaker;
    Translation2d shooterToSpeaker;
    Translation2d shooterToAmp;

    if (currentAlliance.isPresent() && Alliance.Red == currentAlliance.get()) {
      shooterToSpeaker = Constants.FireControl.RED_SPEAKER_POSITION.getTranslation()
          .minus(shooterPose.getTranslation());
          shooterToAmp = Constants.FireControl.RED_AMP_CORNER_POSITION.getTranslation().minus(Constants.FireControl.FEEDOFFSET)
          .minus(shooterPose.getTranslation());// new Transform2d(shooterPose,
                                               // Constants.FireControl.RED_SPEAKER_POSITION).getTranslation();
    } else {
      shooterToSpeaker = Constants.FireControl.BLUE_SPEAKER_POSITION.getTranslation()
          .minus(shooterPose.getTranslation());
      shooterToAmp = Constants.FireControl.BLUE_AMP_CORNER_POSITION.getTranslation().minus(Constants.FireControl.FEEDOFFSET)
          .minus(shooterPose.getTranslation());// new Transform2d(shooterPose,
                                               // Constants.FireControl.BLUE_SPEAKER_POSITION).getTranslation();
    }

    distanceToTarget = distanceFromSpeaker = shooterToSpeaker.getNorm() + Units.inchesToMeters(-6);
    robotDesiredAngle = shooterToSpeaker.getAngle().minus(Rotation2d.fromDegrees(180))
        .plus(Constants.FireControl.AZMUTH_OFFSET);
    feedDesiredAngle = shooterToAmp.getAngle().minus(Rotation2d.fromDegrees(180))
        .plus(Constants.FireControl.AZMUTH_OFFSET);

    shooterAngle = Rotation2d
        .fromRadians(Math.asin(Constants.FireControl.HEIGHT + 0.5 * Constants.FireControl.ACCELERATION));

    shooterVelocity = Constants.FireControl.TARGET_VELOCITY_MPS;
    double shooterConstant = (Constants.FireControl.ACCELERATION * distanceFromSpeaker * distanceFromSpeaker)
        / (2 * shooterVelocity * shooterVelocity);
    double b = distanceFromSpeaker;
    double a = -1 * shooterConstant;
    double c = -1 * (shooterConstant + Constants.FireControl.HEIGHT);
    Rotation2d firstOption = Rotation2d.fromRadians(Math.atan((-b + Math.sqrt(b * b - 4 * a * c)) / (2 * a)));
    Rotation2d secondOption = Rotation2d.fromRadians(Math.atan((-b - Math.sqrt(b * b - 4 * a * c)) / (2 * a)));

    if ((firstOption.getDegrees() >= 0) && (firstOption.getDegrees() <= 85.0)) {
      shooterAngle = firstOption;
    } else if ((secondOption.getDegrees() >= 0) && (secondOption.getDegrees() <= 85.0)) {
      shooterAngle = secondOption;
    } else {
      shooterAngle = Rotation2d.fromDegrees(0);
    }
      


    SmartDashboard.putNumber("FireControl/Distance To Target", distanceFromSpeaker);
    SmartDashboard.putNumber("FireControl/Robot Desired Angle", robotDesiredAngle.getDegrees());
    SmartDashboard.putNumber("FireControl/Angle Error", robotDesiredAngle.getDegrees() - poseSupplier.get().getRotation().getDegrees());

  }

  private double shooterVelocity;
  private Rotation2d shooterAngle;
  private Rotation2d robotDesiredAngle;
  private Rotation2d altShooterAngle;
  private double distanceToTarget;
  private Rotation2d feedDesiredAngle;

  public double getDistanceToTarget() {
    return distanceToTarget;
  }

  public double getVelocity() {
    return shooterVelocity;
  }

  public Rotation2d getAngle() {
    return shooterAngle;
  }

  public Rotation2d getAltAngle() {
    return altShooterAngle;
  }

  public Rotation2d getDesiredRobotAngle() {
    return robotDesiredAngle;
  }

  public void setTargetMode(TargetLocation track) {
    targetingMode = track;
    switch (targetingMode) {
      case AMP:
      case SPEAKER:
      this.trackTarget = true;
        break;
      
      default:
      trackTarget = false;
        break;
    }
    // if (RobotContainer.shooter.hasNote()) {
    }
    // }
    // else {
    // this.trackTarget = false;
    // }

  public boolean trackingTarget() {
    return trackTarget;
  }

public double calculateRotation(Rotation2d targetAngle) {
  double measurement = MathUtil.angleModulus(RobotContainer.swerve.getPose().getRotation().getRadians());
  double target = MathUtil.angleModulus(targetAngle.getRadians());
      if (Math.abs(target - measurement) > Math.PI) {
        if (measurement < (-Math.PI / 2.0)) {
          target -= 2 * Math.PI;
        } else {
          target += 2 * Math.PI;
        }
      }
    rot = speakerController.calculate(measurement, target);
    rot = MathUtil.clamp(rot, -0.5, 0.5);
    SmartDashboard.putNumber("FireControl/Required Rotation", rot);
   return rot;
}
public double getRequiredRotation() {
  switch (targetingMode) {
    case SPEAKER:
      return calculateRotation(getDesiredRobotAngle());
    case AMP:
      return calculateRotation(feedDesiredAngle);
    default:
      return 0;
  }
}

public boolean isAtTargetAngle() {
  //return (Math.abs(robotDesiredAngle.getDegrees() - poseSupplier.get().getRotation().getDegrees()) < 1.0);
  return speakerController.atSetpoint();
}
}
