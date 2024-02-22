package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.FieldCalibration;
import frc.robot.util.Util;
import java.io.IOException;

public final class Constants {
  public static final String canivoreBusName = "rio";
  public static final AprilTagFieldLayout apriltagLayout;

  static {
    try {
      apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      apriltagLayout.getFieldLength();
      apriltagLayout.getFieldWidth();
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final Translation2d fieldSize = new Translation2d(16.54, 8.02);
  public static final double noteTransferMetersPerSecond = 0.5;

  public static final class Shooter {
    public static final int topCANID = 42;
    public static final int bottomCANID = 43;
    public static final int intakeCANID = 40;
    public static final int elevationCANID = 41;
    public static final double gearRatioShooterSide = 1 / 1.333; // Rotations of the motor per rotations of the wheel
    public static final double wheelDiameter = Units.inchesToMeters(3.0);
    public static final double maxRPM = 5700.0;
    public static final double shooterIntakeGearRatio = 60.0 / 1.0;
    public static final double shooterIntakeWheelDiamater = Units.inchesToMeters(3.0);
    public static final double intakeDistancePerMotorRotation = Math.PI / shooterIntakeGearRatio * shooterIntakeWheelDiamater;
     public static final double wristGearRatio = 47.0/18.0*100;
     public static final double ROTATION_DEGREES_PER_ROTATION = 360 / wristGearRatio;
     public static final int maxWristMotorRPM = 6700;
     public static final double maxElevationDegreesPerSecond = maxWristMotorRPM * ROTATION_DEGREES_PER_ROTATION / 60;
     public static final int noteSensorChannel = 1;
     public static final double shootInAmpAngle = -45; //Change pls???
     public static final double meetIntakeAngle = 80.0;
     public static final double allowedErrorInDegreesForAngle = 5.0; ///may change???
     public static final double meetIntake = 80.0;
    public static final int elevationEncoderCANID = 41;
    public static final Rotation2d stowAngle = Rotation2d.fromDegrees(-179.5);
    public static final Rotation2d intakeAngle = Rotation2d.fromDegrees(42.0);
    public static final double shooterIntakeMotorP = 0.00015;
    public static final double shooterIntakeMotorI = 0.00000001;
    public static final double shooterIntakeMotorD = 0.00005;
    public static final double shooterIntakeMotorFeedForward = 0.0001;
    public static final double shooterMotorP = 0.0015;
    public static final double shooterMotorI = 0.000001;
    public static final double shooterMotorD = 0.00;
    public static final double shooterMotorFeedForward = 0.00022;
    public static final double shooterMotorIZone = 200;
    public static final double elevationMotorP = 0.00025;
    public static final double elevationMotorI = 0.0000005;
    public static final double elevationMotorD = 0.000005;
    public static final double elevationMotorFeedForward = 0.0001;
    public static final double elevationMaxVelocity = 4500;
    public static final double elevationMaxAcceleration = 4000;
    public static final double elevationAllowedClosedLoopError = 1 / ROTATION_DEGREES_PER_ROTATION;

  }

  public static final class CameraInfo {
    public static final double cameraHeight = Units.inchesToMeters(0.0); // ?????
    public static final double cameraPitch = Units.degreesToRadians(0); // ?????
  }
  public static final class Intake {

      public static final int intakeCANID = 20;
      // public static final int pivotIntakeCANID = 20;
      // public static final int intakeAngleSensor = 20;
      public static final double gearRatio = 80/1;
      public static final double countsPerMotorRevolution =  42;
      public static final double countsPerRevolution = (gearRatio)*(countsPerMotorRevolution);
      public static final double intakeAcceleration = 20.2;
      public static final double degreesPerRevolution = 360.0;
      public static final double intakeWheelDiameter = Units.inchesToMeters(1.0);
      public static final double ROTATION_DEGREES_PER_ROTATION = 360 / (gearRatio);
      public static final double intakeGearRatio = 5/1;
      public static final double upperDistancePerMotorRotation = Math.PI / intakeGearRatio * intakeWheelDiameter;
      public static final double upPositionDegrees = 90;
      public static final double downPositionDegrees = 0;
      public static final int intakeNoteSensorChannel = 0;
      public static final double allowedAngleErrorInDegrees = 1;
      public static final double intakeMotorP = 0.000075;
      public static final double intakeMotorI = 0.00000005;
      public static final double intakeMotorD = 0.0;
      public static final double intakeMotorPFeedForward = 0.0001;
  }
    
  public static final class Swerve {
    public static final int imuCanID = 3;
    public static final double maxVelTele = 4.7;
    public static final double maxAccelTele = 6.0;
    public static final double maxAngularVelTele = Units.degreesToRadians(180);
    public static final double maxAngularAccelTele = Units.degreesToRadians(540);
    public static final double teleAngleHoldFactor = 3.0;
  
    public static final class Odometry {
      public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);
      public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
      public static final Matrix<N3, N1> visionStdDevsTrust = VecBuilder.fill(0.2, 0.2, 0.2);
    }

    public static final class PathFollowing {
      public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
      public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(2.0, 0.0, 0.0);
    }

    public static final class FrontLeftModule {
      public static final int driveMotorCanID = 14;
      public static final int rotationMotorCanID = 10;
      public static final int rotationEncoderCanID = 10;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(11.25),
          Units.inchesToMeters(12.25));
    }

    public static final class FrontRightModule {
      public static final int driveMotorCanID = 17;
      public static final int rotationMotorCanID = 13;
      public static final int rotationEncoderCanID = 13;
      public static Translation2d moduleOffset = new Translation2d(Units.inchesToMeters(11.25),
          -Units.inchesToMeters(12.25));
    }

    public static final class BackLeftModule {
      public static final int driveMotorCanID = 15;
      public static final int rotationMotorCanID = 11;
      public static final int rotationEncoderCanID = 11;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(11.25),
          Units.inchesToMeters(12.25));
    }

    public static final class BackRightModule {
      public static final int driveMotorCanID = 16;
      public static final int rotationMotorCanID = 12;
      public static final int rotationEncoderCanID = 12;
      public static Translation2d moduleOffset = new Translation2d(-Units.inchesToMeters(11.25),
          -Units.inchesToMeters(12.25));
    }
  }

  public static final class AutoBalance {
    public static final PIDConstants BALANCE_CONSTANTS = new PIDConstants(0.3, 0.0, 0.1);
    public static final double maxVelAuto = 0.4;
    public static final double maxVelTele = 0.3;
  }

  public static final class FireControl {
    public static final double FINAL_Y_VELOCITY = 3;
    public static final double ACCELERATION = 9.81;
    public static final double HEIGHT = Units.inchesToMeters(80.13);
    public static final double TARGET_VELOCITY_MPS = 15;
    // public static final double SHOOTER_HEIGHT = 24;
    // public static final double HEIGHT = SPEAKER_HEIGHT - SHOOTER_HEIGHT;
    public static final Pose2d BLUE_SPEAKER_POSITION = new Pose2d(Units.inchesToMeters(0),
        Units.inchesToMeters(218.5), Rotation2d.fromDegrees(0.0));
    public static final Pose2d RED_SPEAKER_POSITION = new Pose2d(Units.inchesToMeters(630),
        Units.inchesToMeters(218.5), Rotation2d.fromDegrees(180));
    // public static final Pose2d RED_SPEAKER_POSITION = new Pose2d(8.3,4.1, Rotation2d.fromDegrees(0));
    public static final Transform2d SHOOTER_OFFSET = new Transform2d(Units.inchesToMeters(-6), Units.inchesToMeters(0),
        Rotation2d.fromDegrees(180));
  }

  public static final class Elevator {
    public static final int ELEVATORMOTOR_ID = 30;
    public static final double METERS_PER_REV = .180;
    public static final int MOTOR_REV_PER_ROTATION = 100;
    public static final double METERS_PER_MOTOR_REV = METERS_PER_REV / MOTOR_REV_PER_ROTATION;
    public static final double MAX_HEIGHT = -89; 
    public static final double MIN_HEIGHT = 0.15; 
    public static final double elevatorMotorP = 0.00005;
    public static final double elevatorMotorI = 0.00;
    public static final double elevatorMotorD = 0.00;
    public static final double elevatorMotorIZone = 5;
    public static final double elevatorMotorFeedForward = 0.0001;
    public static final double elevatorMotorMinVelocity = 0;
    public static final double elevatorMotorMaxVelocity = 5500;
    public static final double elevatorMotorMaxAcceleration = 10000;
    public static final double elevatorMotorClosedLoppError = 2; 
  }

  public static final class LEDStrip {
    public static final int numLEDs = 45;

    public static final int candleID = 4;
    public static final int swerveLED = 0;
    public static final int pinkArmLED = 1;
    public static final int turretLED = 2;
    public static final int intakeLED = 3;
    public static final int jawLED = 4;
    public static final int stealerLED = 5;
    public static final int manhattanLED = 6;
  }
}
