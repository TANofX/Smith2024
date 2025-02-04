// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class ReadyToPassNote extends Command {
  /** Creates a new ReadyToPassNote. */
  public ReadyToPassNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   if (!RobotContainer.shooter.hasNote()) {
    RobotContainer.shooterWrist.setElevation(Constants.Shooter.intakeAngle);
   }
  }

  // Called every time the scheduler runs while the command is sRotation2d.frcheduled.
  @Override
  public void execute() {
    RobotContainer.shooterWrist.setElevation(Constants.Shooter.intakeAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RobotContainer.shooter.hasNote() || RobotContainer.shooterWrist.isAtElevation());
  }
}
