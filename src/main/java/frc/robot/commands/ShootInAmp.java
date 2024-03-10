// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShootInAmp extends Command {
  /** Creates a new ShootInAmp. */
  public ShootInAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.shooterWrist, RobotContainer.elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (RobotContainer.shooter.hasNote()) {
      RobotContainer.shooter.startMotorsForShooter(5); //change pls??? also make constant pls
      //RobotContainer.elevator.elevatorToMaxHeight();
      RobotContainer.elevator.elevatorToHeight(-80);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     RobotContainer.shooterWrist.setElevation(Constants.Shooter.shootInAmpAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.shooterWrist.isAtElevation() && RobotContainer.shooter.atSpeed();
  }
}
