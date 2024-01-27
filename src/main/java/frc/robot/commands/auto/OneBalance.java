package frc.robot.commands.auto;

/*
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.List;
*/
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneBalance extends SequentialCommandGroup {
/*
    public OneBalance(AutoBuilder autoBuilder) {
     List<PathPlannerTrajectory> pathGroup =
        PathPlanner.fromPathFile("1MidBalance",PathConstraints(1,2));

    addCommands(
        Commands.sequence(
                autoBuilder.resetPose(pathGroup.get(0)),
                AutoBuildingBlocks.setIsCubeMode(true),
                RobotContainer.turret
                    .setTurretAngleCommand(0)
                    .raceWith(
                        Commands.sequence(
                            RobotContainer.pinkArm.setArmPositionCommand(0.0, 155).withTimeout(1.0),
                            RobotContainer.pinkArm
                                .setArmPositionCommand(0.4, 155)
                                .withTimeout(1.0))),
                AutoBuildingBlocks.outtakeGamePiece(),
                RobotContainer.pinkArm.setExtensionLengthCommand(0),
                Commands.waitSeconds(0.5),
                autoBuilder.followPathWithEvents(pathGroup.get(0)),
                RobotContainer.swerve.autoBalance(Constants.AutoBalance.maxVelAuto))
            .withTimeout(14.9)
            .andThen(Commands.run(RobotContainer.swerve::lockModules, RobotContainer.swerve)));
  }
*/
}
