// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.input.controllers.XboxControllerWrapper;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.AtRedSubWoofer;
import frc.robot.commands.AutoFireControl;
import frc.robot.commands.CalibrateElevator;
import frc.robot.commands.CancelShooter;
import frc.robot.commands.ClimbPosition;
import frc.robot.commands.ElevateShooter;
import frc.robot.commands.ElevatorToMax;
import frc.robot.commands.ElevatorToMin;
import frc.robot.commands.ExtendElevator;
import frc.robot.commands.FindMotorExtents;
import frc.robot.commands.FireControlWrist;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ManualShooterElevation;
import frc.robot.commands.ReadyToPassNote;
import frc.robot.commands.RetractElevator;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.RobotFaceSpeaker;
import frc.robot.commands.SafePosition;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootInAmp;
import frc.robot.commands.ShootInSpeaker;
import frc.robot.commands.ShooterIntake;
import frc.robot.commands.SwerveDriveWithGamepad;
import frc.robot.commands.TransferNote;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.LEDs.LEDHasNote;
import frc.robot.commands.Notifications;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0, 0.1);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1, 0.1);

  

  // Subsystems
  public static final Swerve swerve = new Swerve();// new Swerve();
  public static final Elevator elevator = new Elevator();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static final ShooterWrist shooterWrist = new ShooterWrist();
  public static final AprilTags aprilTags = new AprilTags();
  public static final FireControl fireControl = new FireControl(swerve::getPose, DriverStation::getAlliance);
  public static final LEDs LEDs = new LEDs();
  // Other Hardware
  public static final PowerDistribution powerDistribution = new PowerDistribution();

  // Vision clients
  // public static final JetsonClient jetson = new JetsonClient();

  public RobotContainer() {
    // swerve.setDefaultCommand(new SwerveDriveWithGamepad());
    SmartDashboard.putData(swerve.zeroModulesCommand());
    configureButtonBindings();
    LEDs.setDefaultCommand(new Notifications());
   
    

    // SmartDashboard.putData(intake.getIntakePivotTuner());
    // SmartDashboard.putData(intake.getIntakeTuner());
    SmartDashboard.putData("Zero Shooter Elevation", shooterWrist.zeroShooterWrist());
    SmartDashboard.putData("Tune Elevation", shooterWrist.getElevationTunerCommand());
    SmartDashboard.putData("Tune Shooter", shooter.getShooterTunerCommand());
    SmartDashboard.putData("Tune Shooter Intake", shooter.getIntakeTunerCommand());
    SmartDashboard.putData("Tune Intake", intake.getIntakeTuner());
    SmartDashboard.putData("Auto Fire Control", new AutoFireControl());
    // SmartDashboard.putData(Commands.runOnce(() -> {
    // intake.updateRotationOffset();}, intake));

    SmartDashboard.putData("Tune Elevator Motor", elevator.getHeightTunerCommand());
    SmartDashboard.putData("Elevator Extents", new FindMotorExtents());

    SmartDashboard.putData("Robot At Center Blue Ring", Commands.runOnce(() -> {
      swerve.resetOdometry(new Pose2d(new Translation2d(2.9, 5.55), Rotation2d.fromDegrees(0)));
    }, swerve));
    SmartDashboard.putData("Robot At Red Speaker", new AtRedSubWoofer());

    // Register Named Commands for pathplanner
    NamedCommands.registerCommand("ShootInSpeaker", new ShootInSpeaker());
    NamedCommands.registerCommand("RunIntake", new IntakeNote());
    NamedCommands.registerCommand("Shoot", new Shoot(false));
    NamedCommands.registerCommand("TansferNote", new TransferNote());
    // NamedCommands.registerCommand("", );

    Autos.init();


    
  }
  

  private void configureButtonBindings() {

    
    driver.LT().onTrue(new SafePosition());
    driver.RB().onTrue(new ClimbPosition());
    driver.LB().onTrue(new ElevatorToMin());
    driver.X().whileTrue(new ReverseIntake());          
    driver.DLeft()
           .onTrue((new ElevateShooter(Constants.Shooter.SHOOT_IN_SPEAKER_AT_SUBWOOFER).alongWith(Commands.runOnce(() -> {
          shooter.startMotorsForShooter(fireControl.getVelocity());
           }, shooter))).andThen(new Shoot(false).andThen(Commands.waitSeconds(.5).andThen(Commands.runOnce(() -> {
           shooter.stopMotors();

           })))));

    
    driver.DRight().onTrue((new ElevateShooter(Constants.Shooter.SHOOT_AT_PODIUM).alongWith(Commands.runOnce(() -> {
      shooter.startMotorsForShooter(fireControl.getVelocity());
   }, shooter))).andThen(new Shoot(false).andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
      shooter.stopMotors();
    })))));
    driver.RT().whileTrue(new ConditionalCommand(new IntakeNote(), (new IntakeNote().alongWith(new ReadyToPassNote())).andThen(new TransferNote()), shooterWrist::isStowed));
    
    
    
        //Commands.waitSeconds(.5).andThen(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
          //shooter.stopMotors();
       // }, shooter))))));
   
    //coDriver.X().onTrue(new ElevatorToMin());
    coDriver.RB().onTrue(new ReadyToPassNote().andThen(new TransferNote()));
    coDriver.LB().onTrue(new CalibrateElevator());
    coDriver.DUp().whileTrue(new ExtendElevator());
    coDriver.DDown().whileTrue(new RetractElevator());
    coDriver.LT().onTrue(new ShootInAmp().andThen(new Shoot(true)).andThen(new ElevatorToMin().alongWith(new CancelShooter())));
    coDriver.RT().onTrue((new RobotFaceSpeaker().raceWith(new ReadyToPassNote().andThen(new TransferNote().andThen(Commands.waitUntil(() -> {return fireControl.isAtTargetAngle();}))).andThen(new FireControlWrist()))).andThen((new ShootInSpeaker()).andThen(new Shoot(false).andThen(new CancelShooter()))));
    coDriver.START();
    coDriver.B().toggleOnTrue(new ManualShooterElevation(coDriver::getRightY));
   
  /*   
        }, shooter))).andThen(new Shoot().andThen(Commands.waitSeconds(0.5).andThen(Commands.runOnce(() -> {
          shooter.stopMotors();

        }))))); */
  }
}
