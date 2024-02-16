// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final CANSparkFlex elevatorMotor = new CANSparkFlex(Constants.Elevator.ELEVATORMOTOR_ID, MotorType.kBrushless);
  private final SparkPIDController elevatorController = elevatorMotor.getPIDController();
  private final SparkLimitSwitch upperLimitSwitch = elevatorMotor.getForwardLimitSwitch(Type.kNormallyClosed);
  private final SparkLimitSwitch lowerLimitSwitch = elevatorMotor.getReverseLimitSwitch(Type.kNormallyClosed);
  
  /** Creates a new Elevator. */
  public Elevator() {
    
  }
  public void extendElevator() {
    elevatorMotor.set(0.25);
  }
  public void retractElevator() {
    elevatorMotor.set(-0.25);
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
    elevatorMotor.getEncoder().setPosition(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
