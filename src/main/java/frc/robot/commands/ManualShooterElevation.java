// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ManualShooterElevation extends Command {
  private Supplier<Double> control;
    /** Creates a new ManualShooterElevation. */
  public ManualShooterElevation(Supplier<Double> joystickInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    control = joystickInput;
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double degreesPerExecution = Constants.Shooter.maxElevationDegreesPerSecond * .05;
    RobotContainer.shooterWrist.setElevation(Rotation2d.fromDegrees(RobotContainer.shooterWrist.getAbsoluteRotationDegrees() + degreesPerExecution * this.control.get()));
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}