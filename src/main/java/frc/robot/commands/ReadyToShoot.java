// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ReadyToShoot extends Command {
  /** Creates a new ReadyToShoot. */
  public ReadyToShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.shooterWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.startMotorsForShooter(RobotContainer.fireControl.getVelocity());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    RobotContainer.shooterWrist.setElevation(Rotation2d.fromRadians(RobotContainer.fireControl.getAngle().getRadians())); //If calculated Angle not accurate, use the simple one (getAltAngle())
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.shooter.atSpeed() && RobotContainer.shooterWrist.isAtElevation();
  }
}