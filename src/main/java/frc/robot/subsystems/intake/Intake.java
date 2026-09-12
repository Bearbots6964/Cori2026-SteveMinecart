package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.intake.IntakeConstants.DeployMode;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.util.Identifiable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements Identifiable {
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert intakeLeadAlert;
  private final Alert intakeFollowerAlert;

  private final SysIdRoutine routine;

  private final Alert deployLeadAlert;
  private final Alert deployFollowerAlert;

  @AutoLogOutput private IntakeGoal goal = IntakeGoal.IDLE;

  @AutoLogOutput private DeployMode deployMode = DeployMode.STOW;

  @AutoLogOutput private IntakeMode intakeMode = IntakeMode.IDLE;

  @AutoLogOutput
  public final Trigger isDeployed =
      new Trigger(

          // trigger that is true when the intake is fully deployed
          () ->
              (Math.abs(
                      inputs.deployMotorPositionDegrees
                          - IntakeConstants.DeployMotorConstants.DEPLOY_ANGLE.in(Degrees))
                  <= IntakeConstants.DeployMotorConstants.DEPLOY_TOLERANCE.in(Degrees)));

  public Intake(IntakeIO io) {
    this.io = io;

    intakeLeadAlert = new Alert("Intake lead motor disconnected.", Alert.AlertType.kError);
    // critical alert for motor disconnection
    intakeFollowerAlert = new Alert("Intake follower motor disconnected.", Alert.AlertType.kError);
    // critical alert for motor disconnection
    deployFollowerAlert = new Alert("Deploy follower motor disconnected.", Alert.AlertType.kError);
    // critical alert for motor disconnection
    deployLeadAlert = new Alert("Deploy lead motor disconnected.", Alert.AlertType.kError);
    // critical alert for motor disconnection

    routine =
        new SysIdRoutine(
            new Config(
                Volts.per(Second).of(0.5),
                Volts.of(3.0),
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new Mechanism(io::setDeployVoltage, null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);

    intakeLeadAlert.set(!inputs.intakeMotorConnected);
    intakeFollowerAlert.set(!inputs.intakeFollowerMotorConnected);

    deployLeadAlert.set(!inputs.deployMotorConnected);
    deployFollowerAlert.set(!inputs.deployFollowerMotorConnected);

    if (deployMode == DeployMode.DEPLOY && !isDeployed.getAsBoolean()) {
      io.setIntakeMotorVelocity(
          RotationsPerSecond.of(IntakeConstants.intakeMap.get(IntakeMode.REVERSE)));
    } else {
      io.setIntakeMotorVelocity(RotationsPerSecond.of(IntakeConstants.intakeMap.get(intakeMode)));
    }
    io.setDeployPosition(Degrees.of(IntakeConstants.deployMap.get(deployMode)));
    // end of periodic
  }

  public void setGoal(IntakeGoal goal) {
    this.goal = goal;

    switch (goal) {
      case STOW:
        intakeMode = IntakeMode.IDLE;
        deployMode = DeployMode.STOW;
        break;
      case DEPLOY:
        intakeMode = IntakeMode.INTAKE;
        deployMode = DeployMode.DEPLOY;
        break;
      case EJECT:
        intakeMode = IntakeMode.REVERSE;
        break;
      case IDLE:
        intakeMode = IntakeMode.IDLE;
        break;
    }
  }

  public Command setGoalCommand(IntakeGoal goal) {
    return runOnce(() -> setGoal(goal));
  }

  public static enum IntakeGoal {
    // retract the intake back into the frame perimeter
    STOW,
    // deploy intake into deploy position
    DEPLOY,
    // reverse intake, spit out balls, and hold position
    EJECT,
    // stop intake and hold position
    IDLE
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
    return routine.quasistatic(dir);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction dir) {
    return routine.dynamic(dir);
  }
}
