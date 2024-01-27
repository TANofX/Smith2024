package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

public class Test extends SequentialCommandGroup {
  public Test(AutoBuilder autoBuilder, boolean isBlue) {
    PathPlannerPath pathGroup = 
        PathPlannerPath.fromPathFile("Test Path");
    
    addCommands(new InstantCommand( () -> RobotContainer.swerve.resetOdometry(pathGroup.getPreviewStartingHolonomicPose())) ///fix later to april tag positions
                , AutoBuilder.followPath(pathGroup));
  }
}

