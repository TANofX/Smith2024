// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.lib.util.AprilTagStruct;
import frc.lib.vision.AprilTagDetection;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.RobotPoseLookup;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTags extends SubsystemBase {

  final PhotonCamera photonCamera = new PhotonCamera("Global_Shutter_Camera");
  final PhotonCamera photonCamera2 = new PhotonCamera("Arducam_OV9281_USB_Camera");

  final Field2d aprilField = new Field2d();
  // public static Transform3d cameraToRobot = new Transform3d(Units.inchesToMeters(-10.5), Units.inchesToMeters(-12.0),
  //     Units.inchesToMeters(21.5), new Rotation3d(0, Units.degreesToRadians(34.26), Units.degreesToRadians(180)));

  private StructArrayPublisher<AprilTag> aprilTagPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("/AprilTags/Visibilty", new AprilTagStruct()).publish();
  private StructArrayPublisher<AprilTag> aprilTagPublisher2 = NetworkTableInstance.getDefault()
    .getStructArrayTopic("/AprilTags/Visibilty2", new AprilTagStruct()).publish();
   private StructArrayPublisher<AprilTag> aprilTagLayoutPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("/AprilTags/Targets", new AprilTagStruct()).publish();
    /** Creates a new AprilTag. */
  public AprilTags() {
    SmartDashboard.putData("April Field", aprilField);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    final PhotonPipelineResult result = photonCamera.getLatestResult();
    final PhotonPipelineResult result2 = photonCamera2.getLatestResult();



    processResult(result, AprilTagDetection.cameraToRobot);
    processResult(result2, AprilTagDetection.cameraToRobot2);

    ArrayList <AprilTag> allTags = new ArrayList<AprilTag>();
    ArrayList <AprilTag> targetTags = new ArrayList<AprilTag>();
    
    if (result.hasTargets()) {
      var tagsToPublish = result.targets.stream().map(target -> {
        return getTargetPose(RobotContainer.swerve.getPose(), AprilTagDetection.cameraToRobot, target);
      }).toList();

      allTags.addAll(tagsToPublish);
    }
    if (result2.hasTargets()) {
      var tagsToPublish = result2.targets.stream().map(target -> {
        return getTargetPose(RobotContainer.swerve.getPose(), AprilTagDetection.cameraToRobot2, target);
      }).toList();

      allTags.addAll(tagsToPublish);
    }
    aprilTagPublisher.set(allTags.toArray(new AprilTag[allTags.size()]));
    
    for (AprilTag t:allTags) {
      targetTags.add(new AprilTag(t.ID, Constants.apriltagLayout.getTagPose(t.ID).get()));
    }
    aprilTagLayoutPublisher.set(targetTags.toArray(new AprilTag[targetTags.size()]));
   }


  private AprilTag getTargetPose(Pose2d robotPose, Transform3d cameraToRobot, PhotonTrackedTarget target) {
    var camToTarget = target.getBestCameraToTarget();
    var camPose = new Pose3d(robotPose).transformBy(cameraToRobot.inverse());
    var targetPose = camPose.transformBy(camToTarget);
    return new AprilTag(target.getFiducialId(), targetPose);
  }

  private void processResult(PhotonPipelineResult result, Transform3d cameraToRobot) {
     if (result.hasTargets()) {
      // final PhotonTrackedTarget bestTarget = result.getBestTarget();
      // // final int id = bestTarget.getFiducialId();
      // double latency = result.getLatencyMillis();
      // double startTime = Timer.getFPGATimestamp();
      // double timerTime = (Timer.getFPGATimestamp() - startTime) * 1000;
      // double timestamp = timerTime - latency;
      // double maxAge = 0.0;

      PhotonTrackedTarget passedTarget = result.getBestTarget();
      Optional<Pose3d> tagPose = Constants.apriltagLayout.getTagPose(passedTarget.getFiducialId());

      if (tagPose.isPresent()) {
        var imageCaptureTime = result.getTimestampSeconds();
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            passedTarget.getBestCameraToTarget(),
            tagPose.get(),
            cameraToRobot);

        RobotPoseLookup<Pose3d> AprilTagLookup = new RobotPoseLookup<Pose3d>();
        AprilTagLookup.addPose(robotPose);
        aprilField.setRobotPose(robotPose.toPose2d());
        if (passedTarget.getPoseAmbiguity() < 0.10) {
          RobotContainer.swerve.odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.25, .25, .5));
          RobotContainer.swerve.odometry.addVisionMeasurement(robotPose.toPose2d(), imageCaptureTime);
        }
        
    }
    }
  } 
}