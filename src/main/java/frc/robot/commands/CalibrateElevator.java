// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class CalibrateElevator extends Command {
  /** Creates a new calibrateElevator. */
  public CalibrateElevator() {
    addRequirements(RobotContainer.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!RobotContainer.elevator.isLowerLimitSwitchPressed()) {
          RobotContainer.elevator.retractElevator();  
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevator.zeroElevator();
    RobotContainer.elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.elevator.isLowerLimitSwitchPressed();
  }
}
