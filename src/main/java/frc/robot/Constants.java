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
      apriltagLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final Translation2d fieldSize = new Translation2d(16.54, 8.02);
public static final double noteTransferMetersPerSecond = 0.5;

  public static final class Shooter {
    public static final int topCANID = 0;
    public static final int bottomCANID = 0;
    public static final int intakeCANID = 0;
    public static final int elevationCANID = 0;
    public static final double gearRatioShooterSide = 1/1.333; //Rotations of the motor per rotations of the wheel
    public static final double wheelDiameter = Units.inchesToMeters(3.0);
    public static final double maxRPM = 5700.0;
    public static final double shooterIntakeGearRatio = 100.0/1.0;
    public static final double shooterIntakeWheelDiamater = Units.inchesToMeters(3.0);
    public static final double intakeDistancePerMotorRotation = Math.PI / shooterIntakeGearRatio * shooterIntakeWheelDiamater;
     public static final double wristGearRatio = 200.0/1.0;
     public static final double ROTATION_DEGREES_PER_ROTATION = 360 / wristGearRatio;
     public static final int noteSensorChannel = 1;

  }

    public static final class Intake {
     public static final int intakeCANID = 0;
      public static final int pivotIntakeCANID = 0;
      public static final int intakeNoteSensorChannel = 0;
      public static final int intakeAngleSensor = 0;
      

      public static final double gearRatio = 80/1;
      public static final double countsPerMotorRevolution =  42;
      public static final double countsPerRevolution = (gearRatio)*(countsPerMotorRevolution);
      public static final double intakeAcceleration = 20.2;
      public static final double degreesPerRevolution = 360.0;
      public static final double intakeWheelDiameter = Units.inchesToMeters(1.0);
      public static final double ROTATION_DEGREES_PER_ROTATION = 360 / (gearRatio);
    
      
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
      public static final int driveMotorCanID = 11;
      public static final int rotationMotorCanID = 15;
      public static final int rotationEncoderCanID = 15;
      public static Translation2d moduleOffset =
          new Translation2d(Units.inchesToMeters(11.25), Units.inchesToMeters(12.25));
    }

    public static final class FrontRightModule {
      public static final int driveMotorCanID = 14;
      public static final int rotationMotorCanID = 18;
      public static final int rotationEncoderCanID = 18;
      public static Translation2d moduleOffset =
          new Translation2d(Units.inchesToMeters(11.25), -Units.inchesToMeters(12.25));
    }

    public static final class BackLeftModule {
      public static final int driveMotorCanID = 12;
      public static final int rotationMotorCanID = 16;
      public static final int rotationEncoderCanID = 16;
      public static Translation2d moduleOffset =
          new Translation2d(-Units.inchesToMeters(11.25), Units.inchesToMeters(12.25));
    }

    public static final class BackRightModule {
      public static final int driveMotorCanID = 13;
      public static final int rotationMotorCanID = 17;
      public static final int rotationEncoderCanID = 17;
      public static Translation2d moduleOffset =
          new Translation2d(-Units.inchesToMeters(11.25), -Units.inchesToMeters(12.25));
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
    //public static final double SHOOTER_HEIGHT = 24;
    //public static final double HEIGHT = SPEAKER_HEIGHT - SHOOTER_HEIGHT;
    public static final Pose2d BLUE_SPEAKER_POSITION = new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(218.5), Rotation2d.fromDegrees(0.0));
    public static final Pose2d RED_SPEAKER_POSITION = new Pose2d(Units.inchesToMeters(323.25), Units.inchesToMeters(218.5), Rotation2d.fromDegrees(180));
    public static final Transform2d SHOOTER_OFFSET = new Transform2d(Units.inchesToMeters(6), Units.inchesToMeters(0), Rotation2d.fromDegrees(0));
  }

  public static final class Elevator {
    public static final int ELEVATORMOTOR_ID = 2;
    public static final double METERS_PER_REV = .180;
    public static final int MOTOR_REV_PER_ROTATION = 100;
    public static final double METERS_PER_MOTOR_REV = METERS_PER_REV/MOTOR_REV_PER_ROTATION;
    public static final double MAX_HEIGHT = .5;
    public static final double MIN_HEIGHT = .25;
    
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
