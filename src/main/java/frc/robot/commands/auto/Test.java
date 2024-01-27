package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.List;
import frc.robot.commands.auto.Autos;

public class Test extends SequentialCommandGroup {
  public Test(AutoBuilder autoBuilder) {
    PathPlannerPath pathTest =
        PathPlannerPath.fromPathFile("New Path");
        addCommands(
            Commands.sequence(
//              autoBuilder.resetPose(pathTest.getPoint(0)),
//              autoBuilder.followPath(pathTest.getPoint(0)));
                autoBuilder.followPath(pathTest)));
  }
}

