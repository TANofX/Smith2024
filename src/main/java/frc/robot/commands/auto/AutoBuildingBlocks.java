package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoBuildingBlocks {
/*
public static Command scoreStartConeBehindWire() {
    return Commands.sequence(
        RobotContainer.jaw.setJawStateCommand(Jaw.State.IDLE),
        Commands.runOnce(() -> RobotContainer.jaw.setDesiredPivotAngle(300)),
        AutoBuildingBlocks.setIsCubeMode(false),
        Commands.parallel(
                RobotContainer.pinkArm.setArmPositionCommand(0.15, 160),
                allianceConditionalCommand(
                    RobotContainer.turret.setTurretAngleCommand(3.5),
                    RobotContainer.turret.setTurretAngleCommand(-3.5)),
                RobotContainer.intake.intakeHoldCommand())
            .withTimeout(1.0),
        outtakeGamePiece(),
        Commands.runOnce(
            () -> {
              RobotContainer.pinkArm.setArmGoalPosition(0.0, 0.0);
              RobotContainer.turret.setDesiredAngle(0.0);
            },
            RobotContainer.pinkArm,
            RobotContainer.turret));
  }

  public static Command scoreStartConeBehindLoading() {
    return Commands.sequence(
        RobotContainer.jaw.setJawStateCommand(Jaw.State.IDLE),
        Commands.runOnce(() -> RobotContainer.jaw.setDesiredPivotAngle(300)),
        AutoBuildingBlocks.setIsCubeMode(false),
        Commands.parallel(
                RobotContainer.pinkArm.setArmPositionCommand(0.17, 160),
                allianceConditionalCommand(
                    RobotContainer.turret.setTurretAngleCommand(-1.0),
                    RobotContainer.turret.setTurretAngleCommand(2.5)),
                RobotContainer.intake.intakeHoldCommand())
            .withTimeout(1.0),
        outtakeGamePiece(),
        Commands.runOnce(
            () -> {
              RobotContainer.pinkArm.setArmGoalPosition(0.0, 0.0);
              RobotContainer.turret.setDesiredAngle(0.0);
            },
            RobotContainer.pinkArm,
            RobotContainer.turret));
  }

  static Command aimTurretToHighCubeLL() {
    return Commands.run(
        () -> {
          DriverStation.Alliance alliance = DriverStation.getAlliance();

          Pose2d robotPose;
          Translation3d targetPose;

          var fid = LimelightHelpers.getFiducialID("limelight-rear");

          if (alliance == DriverStation.Alliance.Red) {
            robotPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-rear");
            targetPose = ScoringTracker.getScoringPos(fid == 1 ? 1 : 7, alliance);
          } else {
            robotPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-rear");
            targetPose = ScoringTracker.getScoringPos(fid == 6 ? 1 : 7, alliance);
          }

          if (robotPose.equals(new Pose2d())) {
            return;
          }

          Rotation2d turretAngleFieldRelative =
              targetPose
                  .toTranslation2d()
                  .minus(robotPose.getTranslation())
                  .getAngle()
                  .plus(Rotation2d.fromDegrees(180));

          if (Math.abs(turretAngleFieldRelative.minus(Rotation2d.fromDegrees(180)).getDegrees())
              > 10) {
            RobotContainer.turret.setDesiredAngleFieldRelative(
                turretAngleFieldRelative, RobotContainer.swerve.getPose());
          }
        },
        RobotContainer.turret);
  }

  static Command aimTurretToMidCubeLL() {
    return Commands.run(
        () -> {
          DriverStation.Alliance alliance = DriverStation.getAlliance();

          Pose2d robotPose;
          Translation3d targetPose;

          var fid = LimelightHelpers.getFiducialID("limelight-rear");

          if (alliance == DriverStation.Alliance.Red) {
            robotPose = LimelightHelpers.getBotPose2d_wpiRed("limelight-rear");
            targetPose = ScoringTracker.getScoringPos(fid == 1 ? 10 : 16, alliance);
          } else {
            robotPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-rear");
            targetPose = ScoringTracker.getScoringPos(fid == 6 ? 10 : 16, alliance);
          }

          if (robotPose.equals(new Pose2d())) {
            return;
          }

          Rotation2d turretAngleFieldRelative =
              targetPose
                  .toTranslation2d()
                  .minus(robotPose.getTranslation())
                  .getAngle()
                  .plus(Rotation2d.fromDegrees(180));

          if (Math.abs(turretAngleFieldRelative.minus(Rotation2d.fromDegrees(180)).getDegrees())
              > 10) {
            RobotContainer.turret.setDesiredAngleFieldRelative(
                turretAngleFieldRelative, RobotContainer.swerve.getPose());
          }
        },  
        RobotContainer.turret);
  }

  static Command pointToPosition(int rowIdx, int redColIdx, int blueColIdx) {
    return Commands.parallel(
        allianceConditionalCommand(
            new PointToPositionAutoOnly(
                ScoringTracker.getScoringPosAlliance(rowIdx, redColIdx, DriverStation.Alliance.Red),
                true),
            new PointToPositionAutoOnly(
                ScoringTracker.getScoringPosAlliance(
                    rowIdx, blueColIdx, DriverStation.Alliance.Blue),
                true)),
        RobotContainer.jaw.setJawStateCommand(Jaw.State.SCORING_POS),
        RobotContainer.intake.intakeCommand());
  }

  static Command outtakeGamePiece() {
    return outtakeGamePiece(0.25);
  }

  static Command outtakeGamePiece(double outtakeTime) {
    return RobotContainer.intake
        .outtakeCommand()
        .withTimeout(outtakeTime)
        .andThen(RobotContainer.intake.stopIntakeCommand());
  }

  static Command setIsCubeMode(boolean isCubeMode) {
    return Commands.runOnce(() -> RobotContainer.setIsCubeMode(isCubeMode));
  }

  static Command stopAimAtGamePiece() {
    return Commands.runOnce(
        () -> {
          Autos.aimAtGamePiece = false;
        });
  }

  static Command aimAtGamePiece() {
    return Commands.runOnce(
        () -> {
          Autos.aimAtGamePiece = true;
        });
  }
*/
  Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

  static Command allianceConditionalCommand(Command redCommand, Command blueCommand) {
    return new ConditionalCommand(redCommand, blueCommand, () -> (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : null) == Alliance.Red);
  }
} 