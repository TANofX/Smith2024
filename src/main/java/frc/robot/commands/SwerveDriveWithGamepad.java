package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveDriveWithGamepad extends Command {
  private final SlewRateLimiter xVelLimiter;
  private final SlewRateLimiter yVelLimiter;
  private final SlewRateLimiter angularVelLimiter;
  private final PIDController speakerController;

  private Rotation2d rotationTarget = null;
  private final double rotationHoldFactor;

  private final boolean aimAtGamePiece;

  public SwerveDriveWithGamepad(boolean aimAtGamePiece) {
    this.xVelLimiter = new SlewRateLimiter(Constants.Swerve.maxAccelTele);
    this.yVelLimiter = new SlewRateLimiter(Constants.Swerve.maxAccelTele);
    this.angularVelLimiter = new SlewRateLimiter(Constants.Swerve.maxAngularAccelTele);
    this.rotationHoldFactor = Constants.Swerve.teleAngleHoldFactor;
    this.aimAtGamePiece = aimAtGamePiece;
    this.speakerController = new PIDController(1.0, 0, 0.01); //MAKE CONSTANTS
    addRequirements(RobotContainer.swerve);
  }

  public SwerveDriveWithGamepad() {
    this(false);
  }

  @Override
  public void initialize() {
    ChassisSpeeds currentSpeeds = RobotContainer.swerve.getCurrentSpeeds();
    Translation2d hack =
        new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
            .rotateBy(RobotContainer.swerve.getPose().getRotation());
    this.xVelLimiter.reset(hack.getX());
    this.yVelLimiter.reset(hack.getY());
    this.angularVelLimiter.reset(currentSpeeds.omegaRadiansPerSecond);

    rotationTarget = null;
  }

  @Override
  public void execute() {
    double x = -RobotContainer.driver.getLeftY();
    x = Math.copySign(x * x, x);
    double y = -RobotContainer.driver.getLeftX();
    y = Math.copySign(y * y, y);
    double rot;
    if (RobotContainer.fireControl.trackingTarget()) {
      double measurement = MathUtil.angleModulus(RobotContainer.swerve.getPose().getRotation().getRadians());
      double target = MathUtil.angleModulus(RobotContainer.fireControl.getDesiredRobotAngle().getRadians());
      if (Math.abs(target - measurement) > Math.PI) {
        if (measurement < (-Math.PI / 2.0)) {
          target -= 2 * Math.PI;
        } else {
          target += 2 * Math.PI;
        }
      }
      rot = speakerController.calculate(measurement, target);
      rot = MathUtil.clamp(rot, -1, 1);
    }
    else {
    rot = -RobotContainer.driver.getRightX();
    rot = Math.copySign(rot * rot, rot);
    }
    double targetAngularVel = rot * Constants.Swerve.maxAngularVelTele;
    boolean targeting = false;
    boolean stop = x == 0 && y == 0 && rot == 0;

    // Only take over game piece aim if driver is not rotating and intake is spinning (we don't have
    // a game piece in intake)
   



      
    

    double xVel = this.xVelLimiter.calculate(x * Constants.Swerve.maxVelTele);
    double yVel = this.yVelLimiter.calculate(y * Constants.Swerve.maxVelTele);
    double angularVel = this.angularVelLimiter.calculate(targetAngularVel);

    if (stop) {
      rotationTarget = null;

      RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, angularVel));
    } else {
      if (!aimAtGamePiece && Math.abs(angularVel) <= 0.01) {
        if (rotationTarget == null) {
          rotationTarget = RobotContainer.swerve.getPose().getRotation();
        }

        if (Math.abs(xVel) > 0.2 || Math.abs(yVel) > 0.2) {
          Rotation2d error = rotationTarget.minus(RobotContainer.swerve.getPose().getRotation());
          angularVel = error.getRadians() * rotationHoldFactor;
        }
      } else {
        rotationTarget = null;
      }

      if (!targeting) {
        RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, angularVel));
      } else {
        if (Math.abs(
                new Translation2d(xVel, yVel)
                    .getAngle()
                    .minus(RobotContainer.swerve.getPose().getRotation())
                    .getDegrees())
            < 45.0) {
          double vel = Math.sqrt((xVel * xVel) + (yVel * yVel));
          RobotContainer.swerve.driveRobotRelative(new ChassisSpeeds(vel, 0.0, angularVel));
        } else {
          RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, angularVel));
        }
      }
    }

   
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
