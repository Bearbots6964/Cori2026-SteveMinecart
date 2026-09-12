package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.IntakeConstants.DeployMotorConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMotorConstants;

public class IntakeIOTalonFX implements IntakeIO {

  // instantiate intake motor and status signals
  private final TalonFX intakeMotor;
  private final TalonFX intakeFollowerMotor;
  private final TalonFX deployMotor;
  private final CANcoder cancoder;
  private final TalonFX deployFollowerMotor;

  // intake motor status signals
  StatusSignal<Angle> intakeMotorPositionRot;
  StatusSignal<AngularVelocity> intakeMotorVelocityRotPerSec;
  StatusSignal<Voltage> intakeMotorAppliedVolts;
  StatusSignal<Current> intakeMotorCurrentAmps;

  StatusSignal<Voltage> intakeMotorFollowerVoltage;

  // deploy motor status signals
  StatusSignal<Angle> deployMotorPositionRot;
  StatusSignal<AngularVelocity> deployMotorVelocityRotPerSec;
  StatusSignal<Voltage> deployMotorAppliedVolts;
  StatusSignal<Current> deployMotorCurrentAmps;

  // create a new request to reuse for setting voltages and using commands
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public IntakeIOTalonFX() {
    cancoder = new CANcoder(DeployMotorConstants.ENCODER_ID);
    tryUntilOk(
        5, () -> cancoder.getConfigurator().apply(DeployMotorConstants.CANCODER_CONFIG, 0.25));

    intakeMotor = new TalonFX(IntakeMotorConstants.INTAKE_LEAD_ID);
    tryUntilOk(
        5,
        () -> intakeMotor.getConfigurator().apply(IntakeMotorConstants.INTAKE_CONFIGURATION, 0.25));

    intakeFollowerMotor = new TalonFX(IntakeMotorConstants.INTAKE_FOLLOW_ID);
    tryUntilOk(
        5,
        () ->
            intakeFollowerMotor
                .getConfigurator()
                .apply(IntakeMotorConstants.INTAKE_CONFIGURATION, 0.25));

    deployMotor = new TalonFX(DeployMotorConstants.DEPLOY_LEAD_ID);
    tryUntilOk(
        5,
        () -> deployMotor.getConfigurator().apply(DeployMotorConstants.DEPLOY_CONFIGURATION, 0.25));

    deployFollowerMotor = new TalonFX(DeployMotorConstants.DEPLOY_FOLLOW_ID);
    tryUntilOk(
        5,
        () ->
            deployFollowerMotor
                .getConfigurator()
                .apply(DeployMotorConstants.DEPLOY_CONFIGURATION, 0.25));

    // create status signals for intake motor
    intakeMotorPositionRot = intakeMotor.getPosition();
    intakeMotorVelocityRotPerSec = intakeMotor.getVelocity();
    intakeMotorAppliedVolts = intakeMotor.getMotorVoltage();
    intakeMotorCurrentAmps = intakeMotor.getSupplyCurrent();

    // create status signals for deploy motor
    deployMotorPositionRot = deployMotor.getPosition();
    deployMotorVelocityRotPerSec = deployMotor.getVelocity();
    deployMotorAppliedVolts = deployMotor.getMotorVoltage();
    deployMotorCurrentAmps = deployMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeMotorPositionRot,
        intakeMotorVelocityRotPerSec,
        intakeMotorAppliedVolts,
        intakeMotorCurrentAmps,
        deployMotorPositionRot,
        deployMotorVelocityRotPerSec,
        deployMotorAppliedVolts,
        deployMotorCurrentAmps);

    ParentDevice.optimizeBusUtilizationForAll(
        intakeMotor, deployMotor, intakeFollowerMotor, deployFollowerMotor);

    intakeFollowerMotor.setControl(
        new Follower(IntakeMotorConstants.INTAKE_LEAD_ID, MotorAlignmentValue.Opposed));
    deployFollowerMotor.setControl(
        new Follower(DeployMotorConstants.DEPLOY_LEAD_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorConnected =
        BaseStatusSignal.refreshAll(
                intakeMotorPositionRot,
                intakeMotorVelocityRotPerSec,
                intakeMotorAppliedVolts,
                intakeMotorCurrentAmps)
            .isOK();

    inputs.deployMotorConnected =
        BaseStatusSignal.refreshAll(
                deployMotorPositionRot,
                deployMotorVelocityRotPerSec,
                deployMotorAppliedVolts,
                deployMotorCurrentAmps)
            .isOK();
    inputs.intakeFollowerMotorConnected =
        BaseStatusSignal.refreshAll(intakeMotorFollowerVoltage).isOK();

    // update the logged inputs with the latest values from the status signals
    inputs.intakeMotorPositionDegrees = intakeMotorPositionRot.getValueAsDouble() * 360.0;
    inputs.intakeMotorVelocityDegreesPerSecond =
        intakeMotorVelocityRotPerSec.getValueAsDouble() * 360.0;
    inputs.intakeMotorAppliedVolts = intakeMotorAppliedVolts.getValueAsDouble();
    inputs.intakeMotorCurrentAmps = intakeMotorCurrentAmps.getValueAsDouble();
    inputs.deployMotorPositionDegrees = deployMotorPositionRot.getValueAsDouble() * 360.0;
    inputs.deployMotorVelocityDegreesPerSecond =
        deployMotorVelocityRotPerSec.getValueAsDouble() * 360.0;
    inputs.deployMotorAppliedVolts = deployMotorAppliedVolts.getValueAsDouble();
    inputs.deployMotorCurrentAmps = deployMotorCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setDeployPosition(Angle angle) {
    deployMotor.setControl(motionMagicRequest.withPosition(angle).withSlot(0));
  }

  @Override
  public void setIntakeVoltage(Voltage volts) {
    intakeMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setIntakeMotorVelocity(AngularVelocity velocity) {
    intakeMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setDeployVoltage(Voltage volts) {
    deployMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stopIntake() {
    intakeMotor.setControl(neutralOut);
  }

  @Override
  public void stopDeploy() {
    deployMotor.setControl(neutralOut);
  }
}
