// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FireControl extends SubsystemBase {
  private Supplier<Pose2d> poseSupplier;
  private Supplier<Optional<Alliance>> allianceSupplier;

  /** Creates a new FireControl. */
  public FireControl(Supplier<Pose2d> poseSupplier, Supplier<Optional<Alliance>> allianceSupplier) {
    this.poseSupplier = poseSupplier;
    this.allianceSupplier = allianceSupplier;
  }

  @Override
  public void periodic() {
    Pose2d currentPosition = poseSupplier.get();
    Transform2d fieldToRobot = new Transform2d(new Pose2d(), currentPosition); // position of robot (center) from field
    Transform2d shooterTranslation = fieldToRobot.plus(Constants.FireControl.SHOOTER_OFFSET); // shooter offset from the
                                                                                              // robot
    Pose2d shooterPose = new Pose2d(shooterTranslation.getX(), shooterTranslation.getY(),
        shooterTranslation.getRotation());
    Rotation2d angleOffset = Rotation2d.fromDegrees(180);

    double distanceFromSpeaker;
//    Rotation2d robotDesiredAngle;
    Optional<Alliance> currentAlliance = allianceSupplier.get();

    Transform2d shooterToSpeaker;

    if (currentAlliance.isPresent() && Alliance.Red == currentAlliance.get()) {
      shooterToSpeaker = new Transform2d(shooterPose, Constants.FireControl.RED_SPEAKER_POSITION);
    } else {
      shooterToSpeaker = new Transform2d(shooterPose, Constants.FireControl.BLUE_SPEAKER_POSITION);
    }

    System.out.println(shooterToSpeaker);
    distanceFromSpeaker = shooterToSpeaker.getTranslation().getNorm();
    robotDesiredAngle = shooterToSpeaker.getTranslation().getAngle().minus(angleOffset);

    double yShooterVelocity = Math.sqrt(2 * Constants.FireControl.ACCELERATION * Constants.FireControl.HEIGHT); // finding
                                                                                                                // y
                                                                                                                // vector
                                                                                                                // of
                                                                                                                // the
                                                                                                                // shooter's
                                                                                                                // velocity
    double xShootervelocity = (Constants.FireControl.ACCELERATION * distanceFromSpeaker) / yShooterVelocity; // finding
                                                                                                             // x vector
                                                                                                             // of the
                                                                                                             // shooter's
                                                                                                             // velocity

    shooterVelocity = Math
        .sqrt((2 * Constants.FireControl.ACCELERATION * Constants.FireControl.HEIGHT) + Math.pow(xShootervelocity, 2)); // vector
                                                                                                                        // sum
                                                                                                                        // to
                                                                                                                        // find
                                                                                                                        // overall
                                                                                                                        // shooter
                                                                                                                        // velocity
    shooterAngle = Rotation2d.fromRadians(Math.atan(Constants.FireControl.ACCELERATION * distanceFromSpeaker)); // Finding the angle the shooter
                                                                                        // needs to shoot into the
                                                                                        // speaker from its current
                                                                                        // position

    // SmartDashboard.getNumber("shooterAngle", shooterAngle);
    // SmartDashboard.getNumber("ShooterVelocity",
    // Units.radiansToDegrees(shooterVelocity));
    // SmartDashboard.getNumber("distanceFromSpeaker", distanceFromSpeaker);

    // This method will be called once per scheduler run

  }

  private double shooterVelocity;
  private Rotation2d shooterAngle;
  private Rotation2d robotDesiredAngle;

  public double getVelocity() {
    return shooterVelocity;
  }

  public Rotation2d getAngle() {
    return shooterAngle;
  }

  public Rotation2d getDesiredRobotAngle() {
    return robotDesiredAngle;
  }


}
