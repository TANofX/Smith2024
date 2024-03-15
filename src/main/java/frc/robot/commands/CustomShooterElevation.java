// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CustomShooterElevation extends Command {
  /** Creates a new FireControlWrist. */
  private Rotation2d desiredElevation;
  public CustomShooterElevation(double desiredElevation) {
    addRequirements(RobotContainer.shooterWrist);
    this.desiredElevation = Rotation2d.fromDegrees(desiredElevation);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooterWrist.setElevation(desiredElevation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterWrist.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
