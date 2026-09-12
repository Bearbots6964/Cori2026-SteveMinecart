package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.EnumMap;

public class IntakeConstants {

  public enum DeployMode {
    STOW,
    DEPLOY
  }

  public enum IntakeMode {
    IDLE,
    INTAKE,
    REVERSE
  }
  // angle in degrees
  public static EnumMap<DeployMode, Double> deployMap = new EnumMap<>(DeployMode.class);
  // velocity in rotations per second
  public static EnumMap<IntakeMode, Double> intakeMap = new EnumMap<>(IntakeMode.class);

  static {
    deployMap.put(DeployMode.DEPLOY, DeployMotorConstants.DEPLOY_ANGLE.in(Degrees));
    deployMap.put(DeployMode.STOW, DeployMotorConstants.RETRACT_ANGLE.in(Degrees));

    intakeMap.put(IntakeMode.IDLE, 0.0);
    intakeMap.put(IntakeMode.INTAKE, IntakeMotorConstants.TARGET_RPS.in(RotationsPerSecond));
    intakeMap.put(
        IntakeMode.REVERSE, IntakeMotorConstants.TARGET_RPS.unaryMinus().in(RotationsPerSecond));
  }

  public static final class IntakeMotorConstants {
    public static final int INTAKE_LEAD_ID = 0; // update this later
    public static final int INTAKE_FOLLOW_ID = 0;
    public static final double GEAR_RATIO = 24.0 / 16.0;
    public static final Voltage INTAKE_VOLTAGE = Volts.of(11.0);
    public static final AngularVelocity TARGET_RPS = RotationsPerSecond.of(21);

    public static final TalonFXConfiguration INTAKE_CONFIGURATION =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0)
                    .withKG(0)
                    .withKV(0)
                    .withKA(0))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(70.0))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));
  }

  public static final class DeployMotorConstants {
    public static final Angle DEPLOY_ANGLE = Degrees.of(0.0);
    public static final Angle RETRACT_ANGLE = Degrees.of(0.0);
    public static final Angle DEPLOY_TOLERANCE = Degrees.of(5.0);

    public static final int ENCODER_ID = 0;
    public static final int DEPLOY_LEAD_ID = 0;
    public static final int DEPLOY_FOLLOW_ID = 0;

    public static final TalonFXConfiguration DEPLOY_CONFIGURATION =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(
                new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0)
                    .withKG(0)
                    .withKV(0)
                    .withKA(0)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(120.0))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(50)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withFeedbackRemoteSensorID(ENCODER_ID)
                    .withSensorToMechanismRatio(1))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(DEPLOY_ANGLE)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(RETRACT_ANGLE))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0.5)
                    .withMotionMagicAcceleration(1.0));

    public static final CANcoderConfiguration CANCODER_CONFIG =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withMagnetOffset(0.0)
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
  }
}
