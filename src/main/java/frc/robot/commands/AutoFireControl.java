// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FireControl;

public class AutoFireControl extends Command {
  /** Creates a new AutoFireControl. */
private final SlewRateLimiter angularVelLimiter;
  public AutoFireControl() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterWrist, RobotContainer.swerve);
    this.angularVelLimiter =  new SlewRateLimiter(Constants.Swerve.maxAngularAccelTele);
  }

  // Called when the command is initially scheduled.
  
  @Override
  public void initialize() {
    RobotContainer.fireControl.setTargetMode(FireControl.TargetLocation.SPEAKER);
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetAngularVel = RobotContainer.fireControl.getRequiredRotation() * Constants.Swerve.maxAngularVelTele;
    RobotContainer.shooterWrist.setElevation(RobotContainer.fireControl.getAngle());
    RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(0, 0, targetAngularVel));
    
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds());
    RobotContainer.shooterWrist.stopMotor();
    RobotContainer.fireControl.setTargetMode(FireControl.TargetLocation.NONE);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //RobotContainer.shooterWrist.isAtElevation() && RobotContainer.fireControl.isAtTargetAngle();

  }
}
