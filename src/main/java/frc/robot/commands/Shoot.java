// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Shoot extends Command {
  private int loopVar = 0;
  /** Creates a new Shoot. */
  public int desiredState;

  public Shoot(int desiredState) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.elevator);
    this.desiredState = desiredState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopVar = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (desiredState == -1) {
      if (RobotContainer.shooter.readyToShoot() && RobotContainer.elevator.isAtElevation()) {
        loopVar++;
      }
      else {
        loopVar = 0;
      }

    } else if (desiredState == 1) {
       if (RobotContainer.shooter.readyToShoot() && RobotContainer.fireControl.isAtTargetAngle()) {
      loopVar++;
    }
  
    else {
      loopVar = 0;
    }
  }
    else if (desiredState == 0) {
      if(RobotContainer.shooter.readyToShoot()) {
      loopVar = 3;
    }
}    
    SmartDashboard.putNumber("Loop Var", loopVar);
  if (loopVar > 2) {
     RobotContainer.shooter.intakeAtSpeed(-1 * RobotContainer.shooter.getShooterSpeed());
  }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.shooter.hasNote();
  }
}
