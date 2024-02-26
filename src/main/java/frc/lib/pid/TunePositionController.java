// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class TunePositionController extends TuneSparkPIDController {

    public TunePositionController(String motorName, CANSparkBase sparkMotor, Subsystem motorOwner) {
        super(motorName, sparkMotor, motorOwner);
    }

    private double targetPosition;

    @Override
    public void initialize() {
        super.initialize();
        targetPosition = 0;

        SmartDashboard.putNumber(name + " Target Position", targetPosition);
    }

    @Override
    public void execute() {
        super.execute();

        double target = SmartDashboard.getNumber(name + " Target Position", 0.0);
        if (target != targetPosition) { pidController.setReference(target, ControlType.kPosition); targetPosition = target; }
        double error = (target-encoder.getPosition())/target;
        
        SmartDashboard.putNumber(name + " Current Position", encoder.getPosition());
        SmartDashboard.putNumber(name + " Percent Error", error);
    }
}
