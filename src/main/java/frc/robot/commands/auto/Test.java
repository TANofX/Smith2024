package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SwerveDriveWithGamepad;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
/*
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.Autos;
*/

public class Test extends SequentialCommandGroup {
  public Test(AutoBuilder autoBuilder) {
    PathPlannerPath pathGroup = 
        PathPlannerPath.fromPathFile("New Path");
    addCommands(AutoBuilder.followPath(pathGroup));

    // PathPlannerTrajectory trajectory = pathGroup.
    
    // List<PathPlannerTrajectory> pathPlannerTrajectory =
    //     PathPlannerTrajectory(pathGroup, RobotContainer.swerve.getCurrentSpeeds(), RobotContainer.swerve.getPose().getRotation());
    //         Commands.sequence(
    //             autoBuilder.resetPose(),
    //             autoBuilder.followPath(pathGroup.getPoint(0)));
  }
}

