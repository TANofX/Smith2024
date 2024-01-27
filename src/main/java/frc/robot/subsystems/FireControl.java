// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FireControl extends SubsystemBase {
  private Supplier<Pose2d> poseSupplier;
  /** Creates a new FireControl. */
  public FireControl(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
     Pose2d currentPosition = poseSupplier.get();
     Transform2d fieldToRobot = new Transform2d(new Pose2d(), currentPosition);  //position of robot (center) from field
     Transform2d shooterPose = fieldToRobot.plus(Constants.FireControl.SHOOTER_OFFSET);  //shooter offset from the robot
     double distanceFromSpeaker;
      Optional <DriverStation.Alliance> currentAlliance = DriverStation.getAlliance();

     if(currentAlliance.isPresent() && DriverStation.Alliance.Red == DriverStation.getAlliance().get()) {
         distanceFromSpeaker = shooterPose.getTranslation().getDistance(Constants.FireControl.RED_SPEAKER_POSITION.getTranslation()); //Finding the shooter's distance from the robot when on blue alliance
     }
    else {
         distanceFromSpeaker = shooterPose.getTranslation().getDistance(Constants.FireControl.BLUE_SPEAKER_POSITION.getTranslation());//Finding the shooter's distance from the robot when on red alliance
     }

 
     double yShooterVelocity = Math.sqrt(2*Constants.FireControl.ACCELERATION*Constants.FireControl.HEIGHT); //finding y vector of the shooter's velocity
     double xShootervelocity = (Constants.FireControl.ACCELERATION*distanceFromSpeaker)/yShooterVelocity; //finding x vector of the shooter's velocity 

      shooterVelocity = Math.sqrt((2*Constants.FireControl.ACCELERATION*Constants.FireControl.HEIGHT) + Math.pow(xShootervelocity, 2)); //vector sum to find overall shooter velocity
      shooterAngle = Math.atan(Constants.FireControl.ACCELERATION*distanceFromSpeaker); //Finding the angle the shooter needs to shoot into the speaker from its current position

    
     // SmartDashboard.getNumber("shooterAngle", shooterAngle);
     //SmartDashboard.getNumber("ShooterVelocity", Units.radiansToDegrees(shooterVelocity));
     //SmartDashboard.getNumber("distanceFromSpeaker", distanceFromSpeaker);

    // This method will be called once per scheduler run


  }
  private double shooterVelocity;
  private double shooterAngle;

  public double getVelocity() {
    return shooterVelocity;
  }
public double getAngle() {
  return shooterAngle;
}
}
