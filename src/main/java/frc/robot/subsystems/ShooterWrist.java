package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pid.TuneSmartMotionControl;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class ShooterWrist extends AdvancedSubsystem {
  private final CANSparkFlex elevationMotor = new CANSparkFlex(Constants.Shooter.elevationCANID, MotorType.kBrushless);
  private final SparkPIDController elevationController = elevationMotor.getPIDController();
  public final CANcoder elevationEncoder = new CANcoder(Constants.Shooter.elevationEncoderCANID);
  private final CANcoderConfiguration shooterEncoderConfiguration;
  private final StatusSignal<Double> rotationAbsoluteSignal;
  private double targetElevation;

  /** Creates a new Shooter. */
  public ShooterWrist() {
    elevationMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    elevationMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    elevationMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(180.0 / Constants.Shooter.ROTATION_DEGREES_PER_ROTATION));
    elevationMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(-65.0 / Constants.Shooter.ROTATION_DEGREES_PER_ROTATION));
    registerHardware("Elevation Motor", elevationMotor);
    registerHardware("Elevation Encoder", elevationEncoder);
    shooterEncoderConfiguration = new CANcoderConfiguration();
    shooterEncoderConfiguration.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    shooterEncoderConfiguration.MagnetSensor.SensorDirection =
        SensorDirectionValue.Clockwise_Positive;
    shooterEncoderConfiguration.MagnetSensor.MagnetOffset =
        Preferences.getDouble("intakeRotationOffset", 0.0) / 360.0;
    elevationEncoder.getConfigurator().apply(shooterEncoderConfiguration);
    rotationAbsoluteSignal = elevationEncoder.getAbsolutePosition();
    rotationAbsoluteSignal.refresh();
    syncRotationEncoders();
    elevationController.setP(Constants.Shooter.elevationMotorP, 0);
    elevationController.setI(Constants.Shooter.elevationMotorI, 0);
    elevationController.setD(Constants.Shooter.elevationMotorD, 0);
    elevationController.setFF(Constants.Shooter.elevationMotorFeedForward, 0);
    elevationController.setSmartMotionMaxAccel(Constants.Shooter.elevationMaxAcceleration, 0);
    elevationController.setSmartMotionMaxVelocity(Constants.Shooter.elevationMaxVelocity, 0);
    elevationController.setSmartMotionAllowedClosedLoopError(Constants.Shooter.elevationAllowedClosedLoopError, 0);
  }
    
  public void updateRotationOffset() {
    double currentOffset = shooterEncoderConfiguration.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - rotationAbsoluteSignal.getValue()) % 1.0;
    Preferences.setDouble("intakeRotationOffset", offset * 360.0);
    shooterEncoderConfiguration.MagnetSensor.MagnetOffset = offset;
    elevationEncoder.getConfigurator().apply(shooterEncoderConfiguration);
    syncRotationEncoders();
  }

  /** Sync the relative rotation encoder (falcon) to the value of the absolute encoder (CANCoder) */
  public void syncRotationEncoders() {
    elevationMotor.getEncoder().setPosition(-1 * getAbsoluteRotationDegrees() / Constants.Shooter.ROTATION_DEGREES_PER_ROTATION);
  }
  public double getAbsoluteRotationDegrees() {
    return rotationAbsoluteSignal.getValue() * 360 * (-1.0); 
    //tells us what angle we are at
  }
  public void setElevation(Rotation2d elevation) {
    double actualAngle = elevation.getDegrees() - getAbsoluteRotationDegrees();
    double shooterElevationOffset = actualAngle/ Constants.Shooter.ROTATION_DEGREES_PER_ROTATION;
   double angleOfElevation = rotationAbsoluteSignal.getValue() + shooterElevationOffset;
    targetElevation = elevation.getDegrees();
    elevationController.setReference(angleOfElevation,ControlType.kSmartMotion, 0);
  }
  public boolean isAtElevation () {
    return Math.abs(getAbsoluteRotationDegrees() - targetElevation) <= Constants.Shooter.allowedErrorInDegreesForAngle;
  }
  @Override
  public void periodic() {
    rotationAbsoluteSignal.refresh();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Absolute Angle", rotationAbsoluteSignal.getValue() * 360);
    SmartDashboard.putNumber("Shooter Angle from Motor", elevationMotor.getEncoder().getPosition() * Constants.Shooter.ROTATION_DEGREES_PER_ROTATION);
    SmartDashboard.putNumber("Shooter Target Elevation", targetElevation);
  }

  @Override
  protected Command systemCheckCommand() {

    //throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
    return Commands.runOnce(() -> {}, this);
  }
  public Command getElevationTunerCommand() {
    return new TuneSmartMotionControl("Shooter Elevation", elevationMotor, this);
  }
  
}
