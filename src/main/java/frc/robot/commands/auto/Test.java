package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

public class Test extends SequentialCommandGroup {
  public Test(AutoBuilder autoBuilder, boolean isBlue) {
    PathPlannerPath pathGroup = 
        PathPlannerPath.fromPathFile("New Path");
    
    addCommands(AutoBuilder.followPath(pathGroup));
  }
}

