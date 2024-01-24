// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FireControl extends SubsystemBase {
  private Supplier<Pose2d> poseSupplier;
  /** Creates a new FireControl. */
  public FireControl(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
     Pose2d currentPosition = poseSupplier.get();
     Transform2d fieldToRobot = new Transform2d(new Pose2d(), currentPosition);
     Transform2d shooterPose = fieldToRobot.plus(Constants.FireControl.SHOOTER_OFFSET);
     double distanceFromSpeaker;
     if(DriverStation.Alliance.Blue == DriverStation.getAlliance().get()) {
         distanceFromSpeaker = shooterPose.getTranslation().getDistance(Constants.FireControl.BLUE_SPEAKER_POSITION.getTranslation());
     }
    else {
         distanceFromSpeaker = shooterPose.getTranslation().getDistance(Constants.FireControl.RED_SPEAKER_POSITION.getTranslation());
     }

 
     double yShooterVelocity = Math.sqrt(2*Constants.FireControl.ACCELERATION*Constants.FireControl.HEIGHT);
     double xShootervelocity = (Constants.FireControl.ACCELERATION*distanceFromSpeaker)/yShooterVelocity;

     double shooterVelocity = Math.sqrt(2*Constants.FireControl.ACCELERATION*Constants.FireControl.HEIGHT + Math.pow(xShootervelocity, 2));
     double shooterAngle = Math.atan(Constants.FireControl.ACCELERATION*distanceFromSpeaker);

    // This method will be called once per scheduler run
  }
}
