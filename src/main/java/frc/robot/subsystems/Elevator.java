// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.pid.TuneSmartMotionControl;
import frc.lib.pid.TuneVelocitySparkPIDController;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final CANSparkFlex elevatorMotor = new CANSparkFlex(Constants.Elevator.ELEVATORMOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = elevatorMotor.getEncoder();
  private final SparkPIDController elevatorController = elevatorMotor.getPIDController();
  private final SparkLimitSwitch upperLimitSwitch = elevatorMotor.getReverseLimitSwitch(Type.kNormallyClosed);
  private final SparkLimitSwitch lowerLimitSwitch = elevatorMotor.getForwardLimitSwitch(Type.kNormallyClosed);

  
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor.setInverted(false);

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
    elevatorController.setReference(Constants.Elevator.MIN_HEIGHT/Constants.Elevator.METERS_PER_MOTOR_REV, ControlType.kPosition);
  }
  public void elevatorToMaxHeight() {
    elevatorController.setReference(Constants.Elevator.MAX_HEIGHT/Constants.Elevator.METERS_PER_MOTOR_REV, ControlType.kPosition);
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
  }

  public Command getHeightTunerCommand() {
    return new TuneSmartMotionControl("Elevator Elevation", elevatorMotor, this);
  }
  

    
}
