package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.vision.limelight.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.Map;

public final class Autos {
//  private static HashMap<String, Command> eventMap;
  public static AutoBuilder autoBuilder;
  public static ReplanningConfig replanningConfig;
  public static final SendableChooser<Command> autoChooser = new SendableChooser<>();

  static boolean aimAtGamePiece = false;

  public static void init() {
    //eventMap = buildEventMap();

    AutoBuilder.configureHolonomic(
            RobotContainer.swerve::getPose,
            RobotContainer.swerve::resetOdometry,
            RobotContainer.swerve::getCurrentSpeeds,
            RobotContainer.swerve::driveRobotRelative,
            new HolonomicPathFollowerConfig(
              Constants.Swerve.PathFollowing.TRANSLATION_CONSTANTS,
              Constants.Swerve.PathFollowing.ROTATION_CONSTANTS,
              4.5,
              .43,
              replanningConfig = new ReplanningConfig(false,false)
              ),
              () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              RobotContainer.swerve);

    AutoBuilder autoBuilder = new AutoBuilder();

    autoChooser.setDefaultOption("None", none());
    autoChooser.addOption("Test", new Test(autoBuilder));
    
    SmartDashboard.putData("Autonomous Mode", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static Command none() {
    return Commands.none();
  }
/*
  public static Command twoPlusBalanceLoading() {
    return AutoBuildingBlocks.allianceConditionalCommand(
        new TwoPlusBalanceLoading(autoBuilder, false),
        new TwoPlusBalanceLoading(autoBuilder, true));
  }

  public static Command twoPlusBalanceWire() {
    return AutoBuildingBlocks.allianceConditionalCommand(
        new TwoPlusBalanceWire(autoBuilder, false), new TwoPlusBalanceWire(autoBuilder, true));
  }

  public static Command threeWire() {
    return AutoBuildingBlocks.allianceConditionalCommand(
        new ThreePieceWire(autoBuilder, false), new ThreePieceWire(autoBuilder, true));
  }

  public static Command threeLoading() {
    return AutoBuildingBlocks.allianceConditionalCommand(
        new ThreeLoading(autoBuilder, false), new ThreeLoading(autoBuilder, true));
  }

  private static HashMap<String, Command> buildEventMap() {
    return new HashMap<>(
        Map.ofEntries(
            Map.entry("outtake", RobotContainer.intake.outtakeCommand()),
            Map.entry(
                "preAimMidConeLoading",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0, 24.5)
                    .withTimeout(1.0)
                    .andThen(
                        AutoBuildingBlocks.allianceConditionalCommand(
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(-23),
                                RobotContainer.pinkArm.setArmPositionCommand(0, 24.5)),
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(23),
                                RobotContainer.pinkArm.setArmPositionCommand(0, 24.5))))),
            Map.entry(
                "preAimMidConeLoading2",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0.75, 24.5)
                    .withTimeout(1.0)
                    .andThen(
                        AutoBuildingBlocks.allianceConditionalCommand(
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(-23),
                                RobotContainer.pinkArm.setArmPositionCommand(0.74, 24.5)),
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(23),
                                RobotContainer.pinkArm.setArmPositionCommand(0.74, 24.5))))),
            Map.entry(
                "preAimMidCubeLoading",
                Commands.parallel(
                    RobotContainer.turret.setTurretAngleCommand(0),
                    RobotContainer.pinkArm.setArmPositionCommand(0, 19))),
            Map.entry(
                "preAimMidConeWire",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0, 25)
                    .withTimeout(1.0)
                    .andThen(
                        AutoBuildingBlocks.allianceConditionalCommand(
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(24),
                                RobotContainer.pinkArm.setArmPositionCommand(0, 25)),
                            Commands.parallel(
                                RobotContainer.turret.setTurretAngleCommand(-24),
                                RobotContainer.pinkArm.setArmPositionCommand(0, 25))))),
            Map.entry(
                "preAimHighCube",
                Commands.parallel(
                    RobotContainer.turret.setTurretAngleCommand(0),
                    RobotContainer.pinkArm.setArmPositionCommand(0, 30))),
            Map.entry("armToSafePos", new ArmToSafePos()),
            Map.entry(
                "armToZero",
                Commands.parallel(
                    RobotContainer.pinkArm.setArmPositionCommand(0, -10),
                    RobotContainer.turret.setTurretAngleCommand(0))),
            Map.entry(
                "cheeseCube",
                RobotContainer.intake
                    .setIntakeSpeedCommand(-1.0)
                    .withTimeout(0.5)
                    .andThen(RobotContainer.intake.stopIntakeCommand())),
            Map.entry(
                "intakeBehind",
                new ArmToFloorIntakeBehindPos()
                    .alongWith(
                        RobotContainer.jaw
                            .setJawStateCommand(Jaw.State.FLOOR_BEHIND_PICKUP)
                            .andThen(RobotContainer.intake.intakeCommand()))),
            Map.entry(
                "intakeFront",
                Commands.sequence(
                    Commands.runOnce(
                        () -> RobotContainer.setIntakeMode(Constants.IntakeMode.FLOOR)),
                    new SmartIntake(false))),
            Map.entry(
                "preAimHighCubeBehind",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0.0, 155)
                    .alongWith(RobotContainer.turret.setTurretAngleCommand(0))),
            Map.entry(
                "preAimHighCubeBehind2",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0.4, 155)
                    .alongWith(AutoBuildingBlocks.aimTurretToHighCubeLL())),
            Map.entry("aimAtGamePiece", AutoBuildingBlocks.aimAtGamePiece()),
            Map.entry(
                "preAimMidCubeBehind", RobotContainer.pinkArm.setArmPositionCommand(0.0, 162)),
            Map.entry(
                "preAimMidCubeBehind2",
                RobotContainer.pinkArm
                    .setArmPositionCommand(0.0, 162)
                    .alongWith(AutoBuildingBlocks.aimTurretToMidCubeLL())),
            Map.entry("intakeConeSlow", RobotContainer.intake.intakeCommand())));
  }
  */
}

