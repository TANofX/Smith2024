// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.pid.TuneSmartMotionControl;
import frc.lib.pid.TuneVelocitySparkPIDController;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.util.NoteSensor;

public class Intake extends AdvancedSubsystem {

  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.intakeCANID, MotorType.kBrushless);
  //private final CANSparkMax pivotIntakeMotor = new CANSparkMax(Constants.Intake.pivotIntakeCANID, MotorType.kBrushless);
 // private final SparkPIDController pivotController = pivotIntakeMotor.getPIDController();
  private final SparkPIDController intakeController = intakeMotor.getPIDController();
  private final NoteSensor intakeBeamBreakSensor = new NoteSensor(Constants.Intake.intakeNoteSensorChannel);
  
  //private final SparkLimitSwitch downLimitSwitch = pivotIntakeMotor.getReverseLimitSwitch(Type.kNormallyOpen);
  //private final SparkLimitSwitch upLimitSwitch = pivotIntakeMotor.getForwardLimitSwitch(Type.kNormallyClosed);

  /** Creates a new Intake. */
  public Intake() {
   
    registerHardware("Intake Motor", intakeMotor);
   // registerHardware("Pivot Intake Motor", pivotIntakeMotor);

    intakeController.setFF(1/6700, 0);

  }
  public void reverseIntake() {
    intakeController.setReference(-2000, ControlType.kVelocity);
  }
  public void intakeGamePiece() {
    intakeController.setReference(2000, ControlType.kVelocity);
  }

  public void passGamePiece(double speedMetersPerSecond) {
    double speedInRPM = speedMetersPerSecond / (Math.PI * Constants.Intake.intakeWheelDiameter) * 60.0
        * Constants.Intake.intakeGearRatio;

    intakeController.setReference(speedInRPM, ControlType.kVelocity);
  }

  public double getIntakeSpeed() {
    return (Constants.Intake.upperDistancePerMotorRotation * intakeMotor.getEncoder().getVelocity()) * 60;
  }

 
  public void stopIntakeMotor() {
    intakeController.setReference(0, ControlType.kVelocity);
    intakeMotor.stopMotor();
  }

  public void runIntakeMotor(double percentVoltage) {
    intakeMotor.set(percentVoltage);
  }

  public boolean hasNote() {
    return intakeBeamBreakSensor.isTriggered();
  }


  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Sensor", hasNote());
    // This method will be called once per scheduler run
  }





  //public boolean onDownLimitSwitch() {
   // return downLimitSwitch.isPressed();
  //}

  //public boolean onUpLimitSwitch() {
    //return upLimitSwitch.isPressed();
  //}

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
       
        Commands.runOnce(
            () -> {
              if (Math.abs(getIntakeSpeed() - 3) < 0.01) {
                addFault(
                    "[System Check] The Intake motors were unable to spin at the correct speed :(",
                    false,
                    true);
              }
              stopIntakeMotor();
            }, this)
        
        
            )
        .until(() -> getFaults().size() > 0);

  }

  public Command getIntakeTuner() {
    return new TuneVelocitySparkPIDController("Intake", intakeMotor, this);
  }

//  public Command getIntakePivotTuner() {
  //  return new TuneSmartMotionControl("Intake Pivot", pivotIntakeMotor, this);
  //}
}
