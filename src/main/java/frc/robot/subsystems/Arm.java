package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Arm implements Subsystem {
    TalonFX m_Arm_L;
    TalonFX m_Arm_R;

    TalonFXConfiguration ArmConfig_L;
    TalonFXConfiguration ArmConfig_R;

    DynamicMotionMagicVoltage positionRequest;
    VoltageOut voltageRequest;
    Follower right_follow_left;

    // Position Range: 0 ~ 26.4

    public Arm() {
        m_Arm_L = new TalonFX(12, "canivore");
        m_Arm_R = new TalonFX(13, "canivore");

        positionRequest = new DynamicMotionMagicVoltage(0, 100, 400, 4000).withEnableFOC(false);
        voltageRequest = new VoltageOut(0).withEnableFOC(false);
        right_follow_left = new Follower(12, true);

        configure();
        resetPosition(0);
    }

    public void configure() {
        ArmConfig_L = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(20)
                                .withSupplyCurrentLimitEnable(true))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(new Slot0Configs()
                        .withKP(2)
                        .withKI(0)
                        .withKD(0.1)
                        .withKS(0.2)
                        .withKV(0.148258)
                        .withKA(0.01)
                        .withKG(0)
                        .withGravityType(GravityTypeValue.Arm_Cosine))
                .withFeedback(new FeedbackConfigs()
                        .withRotorToSensorRatio(0.0119))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(16)
                        .withMotionMagicAcceleration(16)
                        .withMotionMagicJerk(32))
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                        .withVoltageOpenLoopRampPeriod(0.45))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(25.9)
                        .withReverseSoftLimitThreshold(0.5)
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true));
        ArmConfig_R = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(20)
                                .withSupplyCurrentLimitEnable(true))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(new Slot0Configs()
                        .withKP(2)
                        .withKI(0)
                        .withKD(0.1)
                        .withKS(0.2)
                        .withKV(0.148258)
                        .withKA(0.01)
                        .withKG(0)
                        .withGravityType(GravityTypeValue.Arm_Cosine))
                .withFeedback(new FeedbackConfigs()
                        .withRotorToSensorRatio(0.0119))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(16)
                        .withMotionMagicAcceleration(16)
                        .withMotionMagicJerk(32))
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                        .withVoltageOpenLoopRampPeriod(0.45))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(25.9)
                        .withReverseSoftLimitThreshold(0.5)
                        .withForwardSoftLimitEnable(true)
                        .withReverseSoftLimitEnable(true));

        m_Arm_L.getConfigurator().apply(ArmConfig_L);
        m_Arm_R.getConfigurator().apply(ArmConfig_R);
    }

    public void setVoltage(double voltage) {
        m_Arm_L.setControl(voltageRequest.withOutput(voltage));
        m_Arm_R.setControl(voltageRequest.withOutput(voltage));
    }

    public void resetPosition(double position) {
        m_Arm_L.setPosition(position);
        m_Arm_R.setPosition(position);
    }

    public void arm_up_volt(boolean limitSwitch) {
        setVoltage(2);

        if (ArmConfig_L.SoftwareLimitSwitch.ForwardSoftLimitEnable != limitSwitch) {
            ArmConfig_L.SoftwareLimitSwitch.ForwardSoftLimitEnable = limitSwitch;
            m_Arm_L.getConfigurator().apply(ArmConfig_L);
        }
        if (ArmConfig_R.SoftwareLimitSwitch.ForwardSoftLimitEnable != limitSwitch) {
            ArmConfig_R.SoftwareLimitSwitch.ForwardSoftLimitEnable = limitSwitch;
            m_Arm_R.getConfigurator().apply(ArmConfig_R);
        }

        if (m_Arm_L.getPosition().getValueAsDouble() > 26.4 || m_Arm_R.getPosition().getValueAsDouble() > 26.4) {
            resetPosition(26.4);
        }
        if (m_Arm_L.getPosition().getValueAsDouble() < 0 || m_Arm_R.getPosition().getValueAsDouble() < 0) {
            resetPosition(0);
        }
    }

    public void arm_down_volt(boolean limitSwitch) {
        setVoltage(-1);

        if (ArmConfig_L.SoftwareLimitSwitch.ReverseSoftLimitEnable != limitSwitch) {
            ArmConfig_L.SoftwareLimitSwitch.ReverseSoftLimitEnable = limitSwitch;
            m_Arm_L.getConfigurator().apply(ArmConfig_L);
        }
        if (ArmConfig_R.SoftwareLimitSwitch.ReverseSoftLimitEnable != limitSwitch) {
            ArmConfig_R.SoftwareLimitSwitch.ReverseSoftLimitEnable = limitSwitch;
            m_Arm_R.getConfigurator().apply(ArmConfig_R);
        }

        if (m_Arm_L.getPosition().getValueAsDouble() > 26.4 || m_Arm_R.getPosition().getValueAsDouble() > 26.4) {
            resetPosition(26.4);
        }
        if (m_Arm_L.getPosition().getValueAsDouble() < 0 || m_Arm_R.getPosition().getValueAsDouble() < 0) {
            resetPosition(0);
        }
    }

    public void arm_up() {
        setPosition(8);
    }

    public void arm_down() {
        setPosition(0.6);
    }

    public boolean is_up() {
        return m_Arm_L.getPosition().getValueAsDouble() > 3;
    }

    public boolean is_down() {
        return m_Arm_L.getPosition().getValueAsDouble() <= 3;
    }

    public void setPosition(double position) {
        m_Arm_L.setControl(positionRequest.withPosition(position));
        m_Arm_R.setControl(right_follow_left);
    }

    public void stop() {
        setVoltage(0);
    }

    // public void log() {
    //     System.out.println("Arm_L: " + m_Arm_L.getPosition());
    //     System.out.println("Arm_R: " + m_Arm_R.getPosition());
    // }

}
