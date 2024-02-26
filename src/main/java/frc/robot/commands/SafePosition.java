// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SafePosition extends Command {
  /** Creates a new SafePosition. */
  public SafePosition() {
    addRequirements(RobotContainer.shooterWrist, RobotContainer.shooter, RobotContainer.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooterWrist.setElevation(Constants.Shooter.stowAngle);
    RobotContainer.elevator.elevatorToMinHeight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooterWrist.setElevation(Constants.Shooter.stowAngle);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return RobotContainer.shooterWrist.isAtElevation();
  }
}
