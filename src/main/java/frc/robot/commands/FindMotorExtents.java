// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class FindMotorExtents extends Command {
  /** Creates a new FindMotorExtents. */
  private boolean calibrating = false;
  private boolean extending = false;
  private boolean lastExtention = false;
  private int loopVariable;
  private double maxSum;
  private double minSum;
  public FindMotorExtents() {

    addRequirements(RobotContainer.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extending = false;
    if (!RobotContainer.elevator.isLowerLimitSwitchPressed()) {
      RobotContainer.elevator.retractElevator();
      calibrating = true;
    }
    loopVariable = 0;
    RobotContainer.elevator.zeroElevator();

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (calibrating) {
      if (RobotContainer.elevator.isLowerLimitSwitchPressed()) {
        RobotContainer.elevator.zeroElevator();
        RobotContainer.elevator.stopElevator();
        calibrating = false;
        extending = false;
      }
    }
    else {
      if (extending && RobotContainer.elevator.isUpperLimitSwitchPressed()) {
        maxSum += RobotContainer.elevator.getPosition();
        RobotContainer.elevator.retractElevator();
        extending = false;
        loopVariable++;

      }
      else if (!extending && RobotContainer.elevator.isLowerLimitSwitchPressed()) {
        minSum += RobotContainer.elevator.getPosition();
        RobotContainer.elevator.extendElevator();
        extending = true;

      } 
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevator.stopElevator();
    SmartDashboard.putNumber("AverageElevatorMin", minSum/loopVariable);
    SmartDashboard.putNumber("AverageElevatorMax", maxSum/loopVariable);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return loopVariable == 5;
  }
}
