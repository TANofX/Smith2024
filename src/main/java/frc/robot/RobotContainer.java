// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
//import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
//import frc.lib.input.controllers.rumble.RumbleOff;
//import frc.lib.subsystem.AdvancedSubsystem;
//import frc.lib.util.CycleTracker;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.util.CycleTracker;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.robot.commands.CalibrateElevator;
import frc.robot.commands.ElevatorToMax;
import frc.robot.commands.ElevatorToMin;
import frc.robot.commands.ExtendElevator;
import frc.robot.commands.FindMotorExtents;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ManualShooterElevation;
import frc.robot.commands.ReadyToPassNote;
import frc.robot.commands.RetractElevator;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.RobotFaceSpeaker;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SafePosition;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootInAmp;
import frc.robot.commands.ShootInSpeaker;
import frc.robot.commands.ShooterIntake;
import frc.robot.commands.SwerveDriveWithGamepad;
import frc.robot.commands.TransferNote;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  // Subsystems
  public static final Swerve swerve = new Swerve();//new Swerve();
  public static final Elevator elevator = new Elevator();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static final ShooterWrist shooterWrist = new ShooterWrist();
  public static final AprilTags aprilTags = new AprilTags();
  public static final FireControl fireControl = new FireControl(swerve::getPose, DriverStation::getAlliance);

  // Other Hardware
  public static final PowerDistribution powerDistribution = new PowerDistribution();

  // Vision clients
  // public static final JetsonClient jetson = new JetsonClient();

  public RobotContainer() {
    swerve.setDefaultCommand(new SwerveDriveWithGamepad());
    SmartDashboard.putData(swerve.zeroModulesCommand());
    configureButtonBindings();

    //SmartDashboard.putData(intake.getIntakePivotTuner());
    //SmartDashboard.putData(intake.getIntakeTuner());
    SmartDashboard.putData("Zero Shooter Elevation", Commands.runOnce(() -> { shooterWrist.updateRotationOffset();}, shooter));
    SmartDashboard.putData("Tune Elevation", shooterWrist.getElevationTunerCommand());
    SmartDashboard.putData("Tune Shooter", shooter.getShooterTunerCommand());
    SmartDashboard.putData("Tune Shooter Intake", shooter.getIntakeTunerCommand());
    SmartDashboard.putData("Tune Intake", intake.getIntakeTuner());
    //SmartDashboard.putData(Commands.runOnce(() -> { intake.updateRotationOffset();}, intake));

    SmartDashboard.putData("Tune Elevator Motor", elevator.getHeightTunerCommand());
    SmartDashboard.putData("Elevator Extents", new FindMotorExtents());

    SmartDashboard.putData("Robot At Center Blue Ring", Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(new Translation2d(2.9, 5.55), Rotation2d.fromDegrees(0)));}, swerve));

  }

  private void configureButtonBindings() {

    driver.B().onTrue((new ReadyToPassNote()).andThen(new TransferNote()));
    driver.LT().whileTrue(new RunIntake());
    driver.RT().whileTrue(new IntakeNote());
    driver.LB().whileTrue(new ReverseIntake());
    driver.Y().onTrue(new SafePosition());
    driver.A().whileTrue(new RobotFaceSpeaker());
    driver.X().onTrue((new ShootInSpeaker().andThen(new ShooterIntake()).andThen(Commands.waitSeconds(.5))).andThen(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
       shooter.stopMotors();
      }, shooter)))));

    coDriver.X().onTrue(new ElevatorToMin());
    coDriver.A().onTrue(new ElevatorToMax());
    coDriver.B().onTrue(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
       shooter.stopMotors();
      }, shooter))));
     coDriver.LB().onTrue(new CalibrateElevator());
     coDriver.DUp().whileTrue(new ExtendElevator());
     coDriver.DDown().whileTrue(new RetractElevator());
    coDriver.LT().onTrue(new ShootInAmp());
    coDriver.RT().onTrue(new ShootInSpeaker());
    coDriver.Y().toggleOnTrue(new ManualShooterElevation(coDriver::getRightY));
    }
  }