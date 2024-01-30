package frc.lib.subsystem.selfcheck;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import frc.lib.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingSparkBase implements SelfChecking {
  private final String label;
  private CANSparkBase spark;

  public SelfCheckingSparkBase(String label, CANSparkBase spark) {
    this.label = label;
    this.spark = spark;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    ArrayList<SubsystemFault> faults = new ArrayList<>();

    REVLibError err = spark.getLastError();
    if (err != REVLibError.kOk) {
      faults.add(new SubsystemFault(String.format("[%s]: Error: %s", label, err.name())));
    }

    return faults;
  }
}
