package frc.lib.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

import java.util.Optional;

public class AprilTagDetection {
  private final AprilTag tag;
  private final double timestampSeconds;

  public AprilTagDetection(AprilTag tag, double timestampSeconds) {
    this.tag = tag;
    this.timestampSeconds = timestampSeconds;
  }

  public int getTagID() {
    return this.tag.ID;
  }

  public double getTimestampSeconds() {
    return this.timestampSeconds;
  }

  public Transform3d getCameraToTag() {
    return new Transform3d(this.tag.pose.getTranslation(), this.tag.pose.getRotation());
  }

  public Transform3d getRobotToTag(Transform3d cameraToRobot) {
    Transform3d robotToCamera = cameraToRobot.inverse();
    return robotToCamera.plus(getCameraToTag());
  }

  /**
   * Get the estimated pose of the tag on the field
   *
   * @param cameraToRobot Transform from the camera to the robot
   * @param robotPose Current robot pose on the field
   * @return Pose of the tag on the field
   */
  public static Transform3d cameraToRobot = new Transform3d(Units.inchesToMeters(-28.25), Units.inchesToMeters(-11.0), Units.inchesToMeters(-25.375), new Rotation3d(0,Units.degreesToRadians(-34.26),Units.degreesToRadians(180)));
  public static Transform3d cameraToRobot2 = new Transform3d(Units.inchesToMeters(-24.625), Units.inchesToMeters(-12.75), Units.inchesToMeters(-17.5), new Rotation3d(0, Units.degreesToRadians(39), Units.degreesToRadians(0)));

  public Pose3d getEstimatedTagPose(Transform3d cameraToRobot, Pose2d robotPose) {
    Transform3d robotToTag = getRobotToTag(cameraToRobot);
    Transform3d fieldToRobot =
        new Transform3d(
            new Translation3d(robotPose.getX(), robotPose.getY(), 0),
            new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

    Transform3d fieldToTag = fieldToRobot.plus(robotToTag);
    return new Pose3d(fieldToTag.getTranslation(), fieldToTag.getRotation());
  }

  public Optional<Pose3d> getEstimatedRobotPose(
      AprilTagFieldLayout layout, Transform3d robotToCamera) {
    Optional<Pose3d> fieldToTarget = layout.getTagPose(getTagID());
    return fieldToTarget.map(
        pose3d -> ComputerVisionUtil.objectToRobotPose(pose3d, getCameraToTag(), robotToCamera));
  }


  //   // PID constants should be tuned per robot
  // final double LINEAR_P = 0.1;
  // final double LINEAR_D = 0.0;
  // PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  
  // final double ANGULAR_P = 0.1;
  // final double ANGULAR_D = 0.0;
  // PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  // PhotonCamera photonCamera = new PhotonCamera("photonCamera");

  // public PhotonPipelineResult result = photonCamera.getLatestResult();
  // public PhotonTrackedTarget bestTarget = result.getBestTarget();
  // public final int id = bestTarget.getFiducialId();
  // public final Optional<Pose3d> aprilTag = Constants.apriltagLayout.getTagPose(id);
  // final double goalRange = Units.feetToMeters(3); //?????
    
  // Pose3d aprilTagPose = aprilTag.get();
  // double aprilTagZ = aprilTagPose.getZ();
  // double distance =
  //   PhotonUtils.calculateDistanceToTargetMeters(
  //                         Constants.CameraInfo.cameraHeight,
  //                         aprilTagZ,
  //                         Constants.CameraInfo.cameraPitch,
  //                         Units.degreesToRadians(result.getBestTarget().getPitch()));

}
