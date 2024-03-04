// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.pid.TuneSmartMotionControl;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final CANSparkFlex elevatorMotor = new CANSparkFlex(Constants.Elevator.ELEVATORMOTOR_ID,
      MotorType.kBrushless);
  private final RelativeEncoder encoder = elevatorMotor.getEncoder();
  private final SparkPIDController elevatorController = elevatorMotor.getPIDController();
  private final SparkLimitSwitch upperLimitSwitch = elevatorMotor.getReverseLimitSwitch(Type.kNormallyClosed);
  private final SparkLimitSwitch lowerLimitSwitch = elevatorMotor.getForwardLimitSwitch(Type.kNormallyClosed);

  private double elevatorTarget = 0.0;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor.setInverted(false);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorController.setP(Constants.Elevator.elevatorMotorP);
    elevatorController.setI(Constants.Elevator.elevatorMotorI);
    elevatorController.setD(Constants.Elevator.elevatorMotorD);
    elevatorController.setFF(Constants.Elevator.elevatorMotorFeedForward);
    elevatorController.setIZone(Constants.Elevator.elevatorMotorIZone);
    elevatorController.setSmartMotionMinOutputVelocity(Constants.Elevator.elevatorMotorMinVelocity, 0);
    elevatorController.setSmartMotionMaxVelocity(Constants.Elevator.elevatorMotorMaxVelocity, 0);
    elevatorController.setSmartMotionMaxAccel(Constants.Elevator.elevatorMotorMaxAcceleration, 0);
    elevatorController.setSmartMotionAllowedClosedLoopError(Constants.Elevator.elevatorMotorClosedLoppError, 0);
  }

  public void extendElevator() {
    elevatorMotor.set(-0.25);
  }

  public void retractElevator() {
    elevatorMotor.set(0.25);
  }

  public void stopElevator() {
    elevatorMotor.stopMotor();
  }

  public void elevatorToMinHeight() {
    elevatorToHeight(Constants.Elevator.MIN_HEIGHT);
  }

  public void elevatorToMaxHeight() {
    elevatorToHeight(Constants.Elevator.MAX_HEIGHT);
  }

  private void elevatorToHeight(double motorRotations) {
    elevatorTarget = motorRotations;
    elevatorController.setReference(motorRotations, ControlType.kSmartMotion);
  }

  public boolean isUpperLimitSwitchPressed() {
    return upperLimitSwitch.isPressed();
  }

  public boolean isLowerLimitSwitchPressed() {
    return lowerLimitSwitch.isPressed();
  }

  public void zeroElevator() {
    encoder.setPosition(0);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Target", elevatorTarget);
    SmartDashboard.putNumber("Elevator/Position", getPosition());
  }

  public Command getHeightTunerCommand() {
    return new TuneSmartMotionControl("Elevator Elevation", elevatorMotor, this);
  }

}
