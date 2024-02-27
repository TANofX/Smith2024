// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class TwoMotorShooter extends Shooter {
  private final CANSparkFlex bottomMotor = new CANSparkFlex(Constants.Shooter.bottomCANID, MotorType.kBrushless);
  private final SparkPIDController bottomController = bottomMotor.getPIDController();

  /** Creates a new Shooter. */
  public TwoMotorShooter() {
    bottomMotor.setInverted(true);
    registerHardware("Bottom Motor", bottomMotor);

  }

  public void shootGamePiece(double speedInMps, double spin) {
    double speedInRPM = speedInMps / (Math.PI * Constants.Shooter.wheelDiameter) * 60.0
        * Constants.Shooter.gearRatioShooterSide;
    double topAdjustment = 1 + 0.5 * spin;
    double bottomAdjustment = 1 - 0.5 * spin;
    super.startMotorsForShooter(speedInMps * topAdjustment);
    bottomController.setReference(speedInRPM * bottomAdjustment, ControlType.kVelocity);
  }

  public void stopMotors() {
    super.stopMotors();
    bottomMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public Command systemCheckCommand() {
    throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");

  }
}
