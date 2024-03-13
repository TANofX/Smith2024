// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.input.controllers.rumble.RumbleSinWave;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDs;
import frc.robot.subsystems.LEDs.AnimationTypes;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Notifications extends Command {
  /** Creates a new Notifications. */
  public Notifications() {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.shooterWrist.isStowed()) {
    RobotContainer.driver.setRumbleAnimation(new RumbleSinWave(.5));
   }

   else {
    RobotContainer.driver.setRumbleAnimation(new RumbleOff());
   }
    
  
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
