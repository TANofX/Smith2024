// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class Shooter extends AdvancedSubsystem {
  private final CANSparkFlex topMotor = new CANSparkFlex(Constants.Shooter.topCANID, MotorType.kBrushless);
  private final CANSparkFlex bottomMotor = new CANSparkFlex(Constants.Shooter.bottomCANID, MotorType.kBrushless);
  private final CANSparkMax shooterIntakeMotor = new CANSparkMax(Constants.Shooter.intakeCANID, MotorType.kBrushless);
  private final SparkPIDController topController = topMotor.getPIDController();
   private final SparkPIDController bottomController = bottomMotor.getPIDController();
  
  /** Creates a new Shooter. */
  public Shooter() {
    bottomMotor.setInverted(true);
    registerHardware("Top Motor", topMotor);
    registerHardware("Bottom Motor", bottomMotor);
  }

    public void receiveGamePiece() {
      shooterIntakeMotor.set(0.25);
    }
    public void stopIntakeMotor() {
      shooterIntakeMotor.set(0);
    }
    public void shootGamePiece(double speedInMps, double spin) {
     double speedInRPM = speedInMps/(Math.PI * Constants.Shooter.wheelDiameter)*60.0*Constants.Shooter.gearRatio;
     double topAdjustment = 1 + 0.5 * spin;
     double bottomAdjustment = 1- 0.5 * spin;
      topController.setReference(speedInRPM * topAdjustment, ControlType.kVelocity);
      bottomController.setReference(speedInRPM * bottomAdjustment, ControlType.kVelocity);
    }
    public void stopMotors() {
      topMotor.set(0);
      bottomMotor.set(0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected Command systemCheckCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");

  }
}
