package frc.lib.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pid.TuneSmartMotionControl;
import frc.lib.pid.TuneVelocitySparkPIDController;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.RobotContainer;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Implementation for an SDS Mk4 swerve module using RevNeo Vortex with SparkFlex controller */
public class Mk4SwerveModuleProSparkFlex extends AdvancedSubsystem {
  public enum ModuleCode {
    FL,
    FR,
    BL,
    BR
  }

  // Volts to meters/sec
  private static final double DRIVE_KV = 3.4;
  // Volts to meters/sec^2
  private static final double DRIVE_KA = 0.27;

  // Volts to deg/sec
  private static final double ROTATION_KV = 12.0 / 900;
  // Volts to deg/sec^2
  private static final double ROTATION_KA = 0.00006;

  private static final double DRIVE_GEARING = 1.0/6.12;
  private static final double DRIVE_METERS_PER_ROTATION =
      DRIVE_GEARING * Math.PI * Units.inchesToMeters(4.0);
  private static final double ROTATION_DEGREES_PER_ROTATION = (1/(150/7)) * 360.0;

  // M/s - Tune (Apply full output and measure max vel. Adjust KV/KA for sim if needed)
  public static final double DRIVE_MAX_VEL = 4.65;

  private static final double DRIVE_KP = 0.0000004;
  private static final double DRIVE_KI = 0.000001;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_I_ZONE = 1000;
  private static final double DRIVE_FEED_FORWARD = 0.000149;

  private static final double ROTATION_KP = 0.00002;
  private static final double ROTATION_KI = 0.0;
  private static final double ROTATION_KD = 0.0;
  private static final double ROTATION_I_ZONE = 0;
  private static final double ROTATION_FEED_FORWARD = 0.000149;
  private static final double ROTATION_MAX_VELOCITY = 6700;
  private static final double ROTATION_MAX_ACCELERATION = 10000;
  private static final double ROTATION_ERROR = 0.05;

  

  public final ModuleCode moduleCode;
  // private final LinearSystemSim<N1, N1, N1> driveSim;
  // private final LinearSystemSim<N2, N1, N1> rotationSim;

  private final CANSparkFlex driveMotor;
  // private final REVPhysicsSim driveSimState;
  
  private final CANSparkFlex rotationMotor;
  // private final REVPhysicsSim rotationSimState;

  private final CANcoder rotationEncoder;
  private final CANcoderConfiguration rotationEncoderConfig;
  // private final CANcoderSimState rotationEncoderSimState;
  private final StatusSignal<Double> rotationAbsoluteSignal;
  private final StatusSignal<Double> rotationAbsoluteVelSignal;

  private SwerveModuleState targetState = new SwerveModuleState();

  /**
   * Create a Mk4 swerve module
   *
   * @param moduleCode The code representing this module
   * @param driveMotorCanID The CAN ID of the drive motor
   * @param rotationMotorCanID The CAN ID of the rotation motor
   * @param encoderCanID The CAN ID of the rotation CANCoder
   * @param canBus The name of the can bus the devices are connected to.
   */
  public Mk4SwerveModuleProSparkFlex(
      ModuleCode moduleCode,
      int driveMotorCanID,
      int rotationMotorCanID,
      int encoderCanID,
      String canBus) {
    super(moduleCode.name() + "SwerveModule");

    this.moduleCode = moduleCode;

    rotationEncoder = new CANcoder(encoderCanID, canBus);
    rotationEncoderConfig = new CANcoderConfiguration();
    rotationEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    rotationEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    rotationEncoderConfig.MagnetSensor.MagnetOffset =
        Preferences.getDouble(getName() + "RotationOffset", 0.0) / 360.0;
    rotationEncoder.getConfigurator().apply(rotationEncoderConfig);
    // rotationEncoderSimState = rotationEncoder.getSimState();

    driveMotor = new CANSparkFlex(driveMotorCanID, MotorType.kBrushless);
    driveMotor.setInverted(false);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.getPIDController().setP(DRIVE_KP, 0);
    driveMotor.getPIDController().setI(DRIVE_KI, 0);
    driveMotor.getPIDController().setIZone(DRIVE_I_ZONE, 0);
    driveMotor.getPIDController().setD(DRIVE_KD, 0);
    driveMotor.getPIDController().setFF(DRIVE_FEED_FORWARD, 0);
    // driveSimState.addSparkMax(driveMotor, 8.0f, 5500.0f);

    rotationMotor = new CANSparkFlex(rotationMotorCanID, MotorType.kBrushless);
    rotationMotor.setInverted(true);
    rotationMotor.setIdleMode(IdleMode.kBrake);
    rotationMotor.getPIDController().setP(ROTATION_KP);
    rotationMotor.getPIDController().setI(ROTATION_KI);
    rotationMotor.getPIDController().setIZone(ROTATION_I_ZONE);
    rotationMotor.getPIDController().setD(ROTATION_KD);
    rotationMotor.getPIDController().setFF(ROTATION_FEED_FORWARD);
    rotationMotor.getPIDController().setSmartMotionMaxVelocity(ROTATION_MAX_VELOCITY, 0);
    rotationMotor.getPIDController().setSmartMotionMaxAccel(ROTATION_MAX_ACCELERATION, 0);
    rotationMotor.getPIDController().setSmartMotionAllowedClosedLoopError(ROTATION_ERROR, 0);
    // rotationSimState.addSparkMax(rotationMotor, 8.0, 5500.0);

    rotationAbsoluteSignal = rotationEncoder.getAbsolutePosition();
    rotationAbsoluteVelSignal = rotationEncoder.getVelocity();

    // driveSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(DRIVE_KV, DRIVE_KA));
    // rotationSim =
    //     new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(ROTATION_KV, ROTATION_KA));

    registerHardware("Drive Motor", driveMotor);
    registerHardware("Rotation Motor", rotationMotor);
    registerHardware("Rotation Encoder", rotationEncoder);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + "/DriveTemp", driveMotor.getMotorTemperature());
    SmartDashboard.putNumber(getName() + "/RotationTemp", rotationMotor.getMotorTemperature());

    // Refresh cached values in background
    StatusSignal.waitForAll(
        0,
        rotationAbsoluteSignal,
        rotationAbsoluteVelSignal);
  }

  @Override
  public void simulationPeriodic() {
    // driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    // rotationSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    // rotationEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // driveSim.setInput(driveSimState.getMotorVoltage());
    // rotationSim.setInput(rotationSimState.getMotorVoltage());

    // driveSim.update(0.02);
    // rotationSim.update(0.02);

    // double driveVel = driveSim.getOutput(0) / DRIVE_METERS_PER_ROTATION;
    // driveSimState.setRotorVelocity(driveVel);
    // driveSimState.addRotorPosition(driveVel * 0.02);

    // double rotationPos = rotationSim.getOutput(0) / ROTATION_DEGREES_PER_ROTATION;
    // double rotationDeltaPos = rotationPos - rotationPositionSignal.getValue();
    // rotationSimState.addRotorPosition(rotationDeltaPos);
    // rotationSimState.setRotorVelocity(rotationDeltaPos / 0.02);
    // rotationEncoderSimState.setRawPosition(rotationSim.getOutput(0) / 360.0);
    // rotationEncoderSimState.setVelocity(getRotationVelocityDegreesPerSecond() / 360.0);
  }

  /**
   * Set the desired state of this module
   *
   * @param desiredState Desired state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    driveMotor.setIdleMode(IdleMode.kBrake);
    this.targetState = SwerveModuleState.optimize(desiredState, getState().angle);

    // Don't run the motors if the desired speed is less than 5% of the max
    if (Math.abs(desiredState.speedMetersPerSecond) < DRIVE_MAX_VEL * 0.01) {
      stopMotors();
      return;
    }

    double deltaRot = targetState.angle.getDegrees() - getAbsoluteRotationDegrees();
    if (deltaRot > 180) {
      deltaRot -= 360;
    } else if (deltaRot < -180) {
      deltaRot += 360;
    }
    double targetAngle = getRelativeRotationDegrees() + deltaRot;

    SmartDashboard.putNumber(getName() + "/Target Angle", targetAngle);
    SmartDashboard.putNumber(getName() + "/Delta Rotation", deltaRot);
    SmartDashboard.putNumber(getName() + "/Absolute Rotation", getAbsoluteRotationDegrees());
    SmartDashboard.putNumber(getName() + "/Relative Rotation", getRelativeRotationDegrees());
    SmartDashboard.putNumber(getName() + "/Target Rotations", (targetAngle / ROTATION_DEGREES_PER_ROTATION));

    REVLibError driveError = 
      driveMotor.getPIDController().setReference(60*targetState.speedMetersPerSecond / DRIVE_METERS_PER_ROTATION, ControlType.kVelocity, 0);
    if (driveError != REVLibError.kOk) {
      addFault(
        "[Drive Motor]: Status code: "
        + driveError.name()
      );
    }

    REVLibError rotationError =
      rotationMotor.getPIDController().setReference(targetAngle / ROTATION_DEGREES_PER_ROTATION, ControlType.kSmartMotion, 0);
    if (rotationError != REVLibError.kOk) {
      addFault(
        "[Rotation Motor]: Status code: "
          + rotationError.name()
      );
    }
  }

  /** Stop all motors */
  public void stopMotors() {
    // NeutralOut request for coast mode
    driveMotor.stopMotor();
    rotationMotor.stopMotor();
  }

  /**
   * Get the position of the drive motor in meters
   *
   * @return How far this module has driven in meters
   */
  public double getDrivePositionMeters() {
    return driveMotor.getEncoder().getPosition() * DRIVE_METERS_PER_ROTATION;
  }

  /**
   * Get the relative rotation of this module in degrees.
   *
   * @return Relative rotation
   */
  public double getRelativeRotationDegrees() {
    return rotationMotor.getEncoder().getPosition() * ROTATION_DEGREES_PER_ROTATION;
  }

  /**
   * Get the absolute rotation of this module in degrees.
   *
   * @return Absolute rotation
   */
  public double getAbsoluteRotationDegrees() {
    return rotationAbsoluteSignal.getValue() * 360.0;
  }

  /**
   * Get the velocity of the drive motor in meters/sec
   *
   * @return Drive motor velocity
   */
  public double getDriveVelocityMetersPerSecond() {
    return driveMotor.getEncoder().getVelocity() * DRIVE_METERS_PER_ROTATION;
  }

  /**
   * Get the velocity of the rotation motor in deg/sec
   *
   * @return Rotation motor velocity
   */
  public double getRotationVelocityDegreesPerSecond() {
    return rotationMotor.getEncoder().getVelocity() * ROTATION_DEGREES_PER_ROTATION;
  }

  /**
   * Get the rotation of this module as a Rotation2d
   *
   * @return Rotation2d for this module
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getAbsoluteRotationDegrees());
  }

  /**
   * Get the current state of this module
   *
   * @return Current SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getRotation());
  }

  /**
   * Get the current positions of this module
   *
   * @return Current SwerveModulePosition
   */
  public SwerveModulePosition getPositions() {
    return new SwerveModulePosition(getDrivePositionMeters(), getRotation());
  }

  /**
   * Update the rotation offset for this module. This will assume that the current module position
   * should be the new zero.
   */
  public void updateRotationOffset() {
    double currentOffset = rotationEncoderConfig.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - rotationAbsoluteSignal.getValue()) % 1.0;
    Preferences.setDouble(getName() + "RotationOffset", offset * 360.0);
    rotationEncoderConfig.MagnetSensor.MagnetOffset = offset;
    rotationEncoder.getConfigurator().apply(rotationEncoderConfig);
    syncRotationEncoders();
  }

  /** Sync the relative rotation encoder (falcon) to the value of the absolute encoder (CANCoder) */
  public void syncRotationEncoders() {
    rotationMotor.getEncoder().setPosition(getAbsoluteRotationDegrees() / ROTATION_DEGREES_PER_ROTATION);
  }

  public void lockModule() {
    double targetAngle = -45;
    if (moduleCode == ModuleCode.FL || moduleCode == ModuleCode.BR) {
      targetAngle = 45;
    }

    targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(targetAngle));
    targetState = SwerveModuleState.optimize(targetState, getRotation());

    double deltaRot = targetState.angle.getDegrees() - getAbsoluteRotationDegrees();
    if (deltaRot > 180) {
      deltaRot -= 360;
    } else if (deltaRot < -180) {
      deltaRot += 360;
    }
    double angle = getRelativeRotationDegrees() + deltaRot;


    //MAYBE? .setRotorControl
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.stopMotor();
    rotationMotor.getPIDController().setReference(angle / ROTATION_DEGREES_PER_ROTATION, ControlType.kPosition);
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  driveMotor.stopMotor();
                  rotationMotor.set(0.3);
                },
                this),
            Commands.waitSeconds(0.3),
            Commands.runOnce(
                () -> {
                  if (getRotationVelocityDegreesPerSecond() < 20) {
                    addFault(
                        "[System Check] Rotation motor encoder velocity measured too slow",
                        false,
                        true);
                  }
                  if (rotationAbsoluteVelSignal.getValue() * 360 < 20) {
                    addFault(
                        "[System Check] Absolute encoder velocity measured too slow " + (rotationAbsoluteVelSignal.getValue() * 360), false, true);
                  }
                },
                this),
            Commands.run(
                    () -> {
                      double deltaRot = 90 - getAbsoluteRotationDegrees();
                      if (deltaRot > 180) {
                        deltaRot -= 360;
                      } else if (deltaRot < -180) {
                        deltaRot += 360;
                      }
                      double angle = getRelativeRotationDegrees() + deltaRot;
                      rotationMotor.getPIDController().setReference(
                        angle / ROTATION_DEGREES_PER_ROTATION, ControlType.kSmartMotion, 0);
                    },
                    this)
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (getAbsoluteRotationDegrees() < 70 || getAbsoluteRotationDegrees() > 110) {
                    addFault(
                        "[System Check] Rotation Motor did not reach target position", false, true);
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  driveMotor.setIdleMode(IdleMode.kCoast);
                  driveMotor.set(0.1);
                  rotationMotor.stopMotor();
                },
                this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  if (getDriveVelocityMetersPerSecond() < 0.25) {
                    addFault("[System Check] Drive motor encoder velocity too slow", false, true);
                  }
                  driveMotor.setIdleMode(IdleMode.kBrake);
                  driveMotor.stopMotor();
                },
                this),
            Commands.waitSeconds(0.25),
            Commands.run(
                    () -> {
                      double deltaRot = 0 - getAbsoluteRotationDegrees();
                      if (deltaRot > 180) {
                        deltaRot -= 360;
                      } else if (deltaRot < -180) {
                        deltaRot += 360;
                      }
                      double angle = getRelativeRotationDegrees() + deltaRot;
                      rotationMotor.getPIDController().setReference(
                        angle / ROTATION_DEGREES_PER_ROTATION, ControlType.kSmartMotion, 0);
                    },
                    this)
                .withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(getAbsoluteRotationDegrees()) > 20) {
                    addFault("[System Check] Rotation did not reach target position", false, true);
                  }
                },
                this))
        .until(() -> getFaults().size() > 0)
        .andThen(Commands.runOnce(this::stopMotors, this));
  }
  public Command getDriveTunerCommand() {
    return new TuneVelocitySparkPIDController("Drive Motors", driveMotor, this);
  }
  public Command getSteerTunerCommand() {
   return new TuneSmartMotionControl("Steer Motors", rotationMotor, this);
  }
}
