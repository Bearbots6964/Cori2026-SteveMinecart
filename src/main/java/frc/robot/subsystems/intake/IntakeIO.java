package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {

    // make sure motors are connected and working
    public boolean intakeMotorConnected = false;
    public boolean intakeFollowerMotorConnected = false;
    public boolean deployMotorConnected = false;
    public boolean deployFollowerMotorConnected = false;

    // inputs that need to be configured for intake motor
    public double intakeMotorPositionDegrees = 0.0;
    public double intakeMotorVelocityDegreesPerSecond = 0.0;
    public double intakeMotorAppliedVolts = 0.0;
    public double intakeMotorCurrentAmps = 0.0;

    // inputs that need to be configured for deploy motor
    public double deployMotorPositionDegrees = 0.0;
    public double deployMotorVelocityDegreesPerSecond = 0.0;
    public double deployMotorAppliedVolts = 0.0;
    public double deployMotorCurrentAmps = 0.0;
  }

  // update the inputs every 50 hertz, new system core is 5 hertz
  public default void updateInputs(IntakeIOInputs inputs) {}

  // set the postion of the deploy motor based on an angle
  public default void setDeployPosition(Angle angle) {}

  // set intake voltage (used for control)
  public default void setIntakeVoltage(Voltage volts) {}

  // set deploy voltage (used for control)
  public default void setDeployVoltage(Voltage volts) {}

  public default void setIntakeMotorVelocity(AngularVelocity velocity) {}

  // stop deploy
  public default void stopDeploy() {}

  // stop intake
  public default void stopIntake() {}
}
