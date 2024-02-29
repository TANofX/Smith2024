// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ShootInSpeaker extends Command {
  /** Creates a new PassNoteFromIntakeToSpeaker. */
  public ShootInSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.shooter.hasNote()) {
    RobotContainer.shooter.startMotorsForShooter(RobotContainer.fireControl.getVelocity());
    System.out.println("Trying to start Shooter Motors");
    RobotContainer.shooter.setElevation(Rotation2d.fromDegrees(10));
  //RobotContainer.fireControl.setTargetMode(true);
  }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
