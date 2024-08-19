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
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public static final TalonFX m_Arm_L = new TalonFX(12, "canivore");
    public static final TalonFX m_Arm_R = new TalonFX(13, "canivore");

    public static double arm_speaker_target = 1;
    public static double arm_notepass_target = 17;

    public static boolean arm_down_flag = false;

    public static Timer down_timer = new Timer();

    TalonFXConfiguration ArmConfig_L;
    TalonFXConfiguration ArmConfig_R;

    DynamicMotionMagicVoltage positionRequest;
    VoltageOut voltageRequest;
    Follower right_follow_left;

    // Position Range: 0 ~ 26.4

    public Arm() {

        positionRequest = new DynamicMotionMagicVoltage(0, 100, 300, 3000).withEnableFOC(true);
        voltageRequest = new VoltageOut(0).withEnableFOC(true);
        right_follow_left = new Follower(12, true);

        configure();
        // resetPosition(0);
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
                        .withKD(0.2)
                        .withKS(0)
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
                        .withKD(0.2)
                        .withKS(0)
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
        // System.out.println("setVoltage: " + voltage);
    }

    public void resetPosition(double position) {
        m_Arm_L.setPosition(position);
        m_Arm_R.setPosition(position);
        // System.out.println("resetPosition: " + position);
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
        // System.out.println("arm_up_volt: " + limitSwitch);
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
        // System.out.println("arm_down_volt: " + limitSwitch);
    }

    public void arm_up() {
        setPosition(8);
        // System.out.println("arm_up");
    }

    public void arm_up_speaker() {
        setPosition(arm_speaker_target);
        // System.out.println("arm_up_speaker");
    }

    public void arm_up_notepass() {
        setPosition(arm_notepass_target);
        // System.out.println("arm_up_notepass");
    }

    public void arm_down() {
        if (m_Arm_L.getPosition().getValueAsDouble() < 3) {
            stop();
            arm_down_flag = false;
            // System.out.println("down flag false by arm_down");
            return;
        } else {
            setPosition(0.6);
            arm_down_flag = true;
            // System.out.println("down flag true by arm_down");
        }
        // System.out.println("arm_down");
    }

    public void arm_pos_magic(double pos, double max_vel, double accel, double jerk) {
        DynamicMotionMagicVoltage req = new DynamicMotionMagicVoltage(pos, max_vel, accel, jerk).withEnableFOC(true);
        m_Arm_L.setControl(req);
        m_Arm_R.setControl(right_follow_left);
        // System.out.println("arm_pos_magic: " + pos);
    }

    public void arm_vel_magic(double vel, double accel) {
        MotionMagicVelocityVoltage req = new MotionMagicVelocityVoltage(vel).withAcceleration(accel)
                .withEnableFOC(true);
        m_Arm_L.setControl(req);
        m_Arm_R.setControl(right_follow_left);
        // System.out.println("arm_vel_magic: " + vel);
    }

    public void set_angle(double angle) {
        setPosition(angle);
        // System.out.println("set_angle: " + angle);
    }

    public double get_angle() {
        return m_Arm_L.getPosition().getValueAsDouble();
    }

    public boolean is_up() {
        return m_Arm_L.getPosition().getValueAsDouble() > 3;
    }

    public boolean is_amp() {
        return m_Arm_L.getPosition().getValueAsDouble() > 25;
    }

    public boolean is_down() {
        return m_Arm_L.getPosition().getValueAsDouble() <= 3;
    }

    public boolean is_ready(double angle) {
        return Math.abs(m_Arm_L.getPosition().getValueAsDouble() - angle) < 1;
    }

    public void setPosition(double position) {

        if (position > 25.9) {
            position = 25.9;
        }
        if (position < 0.5) {
            position = 0.5;
        }

        m_Arm_L.setControl(positionRequest.withPosition(position));
        m_Arm_R.setControl(right_follow_left);

        // System.out.println("setPosition: " + position);
    }

    public void stop() {
        setVoltage(0);
        // System.out.println("stop");
    }

    @Override
    public void periodic() {
        if (arm_down_flag && m_Arm_L.getPosition().getValueAsDouble() < 3) {
            stop();
            arm_down_flag = false;
            // System.out.println("periodic stop A");
        }
        if (arm_down_flag == true && Math.abs(m_Arm_L.getVelocity().getValueAsDouble()) < 1) {
            down_timer.reset();
            down_timer.start();
            // System.out.println("periodic stop B");
        } else {
            down_timer.stop();
            down_timer.reset();
        }
        if (down_timer.get() > 0.5) {
            stop();
            arm_down_flag = false;
            // System.out.println("periodic stop C");
        }

        if (m_Arm_L.getPosition().getValueAsDouble() > 26.4 || m_Arm_R.getPosition().getValueAsDouble() > 26.4) {
            resetPosition(26.4);
        }
        if (m_Arm_L.getPosition().getValueAsDouble() < 0 || m_Arm_R.getPosition().getValueAsDouble() < 0) {
            resetPosition(0);
        }
    }

}
