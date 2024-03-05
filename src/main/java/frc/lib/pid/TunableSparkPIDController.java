// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.pid;

import com.revrobotics.SparkPIDController;

/** Add your docs here. */
public class TunableSparkPIDController implements TunablePID {
    private final SparkPIDController pidController;
    private final int slot;

    public TunableSparkPIDController(SparkPIDController pid, int pidSlot) {
        pidController = pid;
        slot = pidSlot;
    }

    @Override
    public double getD() {
        return pidController.getD(slot);
    }

    @Override
    public double getFF() {
        return pidController.getFF(slot);
    }

    @Override
    public double getI() {
        return pidController.getI(slot);
    }

    @Override
    public double getIZone() {
        return pidController.getIZone(slot);
    }

    @Override
    public double getOutputMax() {
        return pidController.getOutputMax(slot);
    }

    @Override
    public double getOutputMin() {
        return pidController.getOutputMin(slot);
    }

    @Override
    public double getP() {
        return pidController.getP(slot);
    }

    @Override
    public void setD(double kD) {
        pidController.setD(kD, slot);        
    }

    @Override
    public void setFF(double kFF) {
        pidController.setFF(kFF, slot);
    }

    @Override
    public void setI(double kI) {
        pidController.setI(kI, slot);
    }

    @Override
    public void setIZone(double Izone) {
        pidController.setIZone(Izone, slot);
    }

    @Override
    public void setOutputRange(double minOutput, double maxOutput) {
        pidController.setOutputRange(minOutput, maxOutput, slot);
    }

    @Override
    public void setP(double kP) {
        pidController.setP(kP, slot);     
    }
}

