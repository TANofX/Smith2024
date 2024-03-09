// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.input.controllers.rumble.RumbleAnimation;
import frc.lib.input.controllers.rumble.RumbleSinWave;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class SafePosition extends Command {
  /** Creates a new SafePosition. */
  public SafePosition() {
    addRequirements(RobotContainer.shooterWrist, RobotContainer.elevator, RobotContainer.shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooterWrist.setElevation(Constants.Shooter.stowAngle);
    RobotContainer.elevator.elevatorToMinHeight();
    RobotContainer.shooter.stopMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooterWrist.setElevation(Constants.Shooter.stowAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   if(RobotContainer.shooterWrist.isStowed()) {
    RobotContainer.driver.setRumbleAnimation(new RumbleSinWave(.5));
  
   }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return RobotContainer.shooterWrist.isAtElevation();

  }
}
