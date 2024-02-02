// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.util.CycleTracker;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.SwerveDriveWithGamepad;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  // Subsystems
  public static final Swerve swerve = new Swerve();
  public static final Elevator elevator = new Elevator();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static final FireControl fireControl = new FireControl(swerve::getPose);

  // Other Hardware
  public static final PowerDistribution powerDistribution = new PowerDistribution();

  // Vision clients
  //  public static final JetsonClient jetson = new JetsonClient();

  


  public RobotContainer() {
    swerve.setDefaultCommand(new SwerveDriveWithGamepad());
    SmartDashboard.putData(swerve.zeroModulesCommand());
    configureButtonBindings();
  }
  
private void configureButtonBindings() {
  driver.A().whileTrue(new IntakeNote());
}
  
  }
