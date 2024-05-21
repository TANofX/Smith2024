// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class TransferNote extends Command {
  /** Creates a new TransferNote. */
  public TransferNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake, RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.intake.passGamePiece(0.5);
    RobotContainer.intake.passGamePiece(1);
    RobotContainer.shooter.intakeAtSpeed(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stopIntakeMotor();
    RobotContainer.intake.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return RobotContainer.shooter.hasNote();
  }
}
