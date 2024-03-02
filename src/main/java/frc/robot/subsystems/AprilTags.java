// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.lib.vision.AprilTagDetection;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.RobotPoseLookup;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTag;

public class AprilTags extends SubsystemBase {

  final PhotonCamera photonCamera = new PhotonCamera("Global_Shutter_Camera");
  final Field2d aprilField = new Field2d();

  /** Creates a new AprilTag. */
  public AprilTags() {
    SmartDashboard.putData("April Field", aprilField);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    final PhotonPipelineResult result = photonCamera.getLatestResult();

    if (result.hasTargets()) {
      // final PhotonTrackedTarget bestTarget = result.getBestTarget();
      // final int id = bestTarget.getFiducialId();
      // double latency = result.getLatencyMillis();
      // double startTime = Timer.getFPGATimestamp();
      // double timerTime = (Timer.getFPGATimestamp() - startTime) * 1000;
      // double timestamp = timerTime - latency;
      // double maxAge = 0.0;

      Optional<Pose3d> tagPose = Constants.apriltagLayout.getTagPose(result.getBestTarget().getFiducialId());

      if (tagPose.isPresent()) {
        var imageCaptureTime = result.getTimestampSeconds();
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            result.getBestTarget().getBestCameraToTarget(),
            tagPose.get(),
            AprilTagDetection.cameraToRobot);

        RobotPoseLookup<Pose3d> AprilTagLookup = new RobotPoseLookup<Pose3d>();
        AprilTagLookup.addPose(robotPose);
        aprilField.setRobotPose(robotPose.toPose2d());
        RobotContainer.swerve.odometry.addVisionMeasurement(robotPose.toPose2d(), imageCaptureTime);
      }
    }
  }
}