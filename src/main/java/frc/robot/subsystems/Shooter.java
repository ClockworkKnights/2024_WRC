package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public static TalonFX m_Shooter_D;
    public static TalonFX m_Shooter_U;

    TalonFXConfiguration motorCfg_Shooter_D;
    TalonFXConfiguration motorCfg_Shooter_U;

    public static double shooter_speaker_target = 60;
    public static double shooter_notepass_target = 80;
    public static boolean break_flag = false;

    VelocityTorqueCurrentFOC velocityRequest;
    MotionMagicVelocityTorqueCurrentFOC motionMagicVelRequest;
    VoltageOut voltageRequest;

    public Shooter() {
        m_Shooter_D = new TalonFX(16, "canivore");
        m_Shooter_U = new TalonFX(17, "canivore");

        velocityRequest = new VelocityTorqueCurrentFOC(0);
        voltageRequest = new VoltageOut(0);
        motionMagicVelRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

        configure();
    }

    public void configure() {
        motorCfg_Shooter_D = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(40)
                                .withSupplyCurrentLimitEnable(false))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(new Slot0Configs()
                        .withKP(10)// 2)
                        .withKI(0)
                        .withKD(0.15)
                        .withKS(0)
                        .withKV(0.079603)
                        .withKA(0.622)// 2)
                        .withKG(0))
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                        .withVoltageOpenLoopRampPeriod(0.3))
                // .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                // .withTorqueClosedLoopRampPeriod(0.3))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(250)
                        .withMotionMagicJerk(2500));

        motorCfg_Shooter_U = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(40)
                                .withSupplyCurrentLimitEnable(false))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(new Slot0Configs()
                        .withKP(10)// 2)
                        .withKI(0)
                        .withKD(0.15)
                        .withKS(0)
                        .withKV(0.079603)
                        .withKA(0.622)// 2)
                        .withKG(0))
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                        .withVoltageOpenLoopRampPeriod(0.3))
                // .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                // .withTorqueClosedLoopRampPeriod(0.3))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(250)
                        .withMotionMagicJerk(2500));

        m_Shooter_D.getConfigurator().apply(motorCfg_Shooter_D);
        m_Shooter_U.getConfigurator().apply(motorCfg_Shooter_U);
    }

    private void setVoltage(double voltage) {
        m_Shooter_D.setControl(voltageRequest.withOutput(voltage));
        m_Shooter_U.setControl(voltageRequest.withOutput(voltage));
    }

    // public void setVelocity(double velocity) {
    // m_Shooter_D.setControl(velocityRequest.withVelocity(velocity));
    // m_Shooter_U.setControl(velocityRequest.withVelocity(velocity));
    // }

    public void setMagicVelocity(double velocity, double accel) {
        m_Shooter_D.setControl(motionMagicVelRequest.withVelocity(velocity).withAcceleration(accel));
        m_Shooter_U.setControl(motionMagicVelRequest.withVelocity(velocity).withAcceleration(accel));
    }

    public void shoot_out(double vel) {
        setMagicVelocity(vel, 300);
    }

    public void shoot_autoaim() {
        setMagicVelocity(shooter_speaker_target, 300);
    }

    public void shoot_notepass() {
        setMagicVelocity(shooter_notepass_target, 300);
    }

    public boolean speed_ready_autoaim() {
        return speed_ready(shooter_speaker_target);
    }

    public boolean speed_ready_notepass() {
        return speed_ready(shooter_notepass_target);
    }

    public void shoot_amp() {
        setMagicVelocity(12, 600);
    }

    public void shoot_magic_vel(double vel, double accel) {
        setMagicVelocity(vel, accel);
    }

    public void shoot_break() {
        setMagicVelocity(0, 100);
        break_flag = true;
    }

    public boolean speed_ready(double speed) {
        return Math.abs(m_Shooter_D.getVelocity().getValueAsDouble() - speed) < 0.2;
    }

    public static double get_speed() {
        return m_Shooter_D.getVelocity().getValueAsDouble();
    }

    // public void shoot_out_voltage() {
    // setVoltage(10);
    // }

    // public void shoot_revert_voltage() {
    // setVoltage(-5);
    // }

    public void stop() {
        setVoltage(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (break_flag) {
            if (Math.abs(m_Shooter_D.getVelocity().getValueAsDouble()) < 1) {
                break_flag = false;
                stop();
            }
        }
    }

}
