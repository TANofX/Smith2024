// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.util.CycleTracker;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  // Subsystems
  public static final Swerve swerve = new Swerve();

  // Other Hardware
  public static final PowerDistribution powerDistribution = new PowerDistribution();

  // Vision clients
  //  public static final JetsonClient jetson = new JetsonClient();

  


 
  

  
  }
