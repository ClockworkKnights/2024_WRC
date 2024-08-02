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

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem {
    TalonFX m_Shooter_D;
    TalonFX m_Shooter_U;

    TalonFXConfiguration ShooterConfig_D;
    TalonFXConfiguration ShooterConfig_U;

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
        ShooterConfig_D = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(40)
                                .withSupplyCurrentLimitEnable(true))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(new Slot0Configs()
                        .withKP(10)// 2)
                        .withKI(0)
                        .withKD(0.15)
                        .withKS(3.48574)
                        .withKV(0.079603)
                        .withKA(0.622)// 2)
                        .withKG(0))
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                        .withVoltageOpenLoopRampPeriod(0.3))
                // .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                //         .withTorqueClosedLoopRampPeriod(0.3))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(250)
                        .withMotionMagicJerk(2500));

        ShooterConfig_U = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(40)
                                .withSupplyCurrentLimitEnable(true))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(new Slot0Configs()
                        .withKP(10)// 2)
                        .withKI(0)
                        .withKD(0.15)
                        .withKS(3.48574)
                        .withKV(0.079603)
                        .withKA(0.622)// 2)
                        .withKG(0))
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                        .withVoltageOpenLoopRampPeriod(0.3))
                // .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                //         .withTorqueClosedLoopRampPeriod(0.3))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(16)
                        .withMotionMagicAcceleration(250)
                        .withMotionMagicJerk(2500));

        m_Shooter_D.getConfigurator().apply(ShooterConfig_D);
        m_Shooter_U.getConfigurator().apply(ShooterConfig_U);

    }

    // public void setVoltage(double voltage) {
    //     m_Shooter_D.setControl(voltageRequest.withOutput(voltage));
    //     m_Shooter_U.setControl(voltageRequest.withOutput(voltage));
    // }

    // public void setVelocity(double velocity) {
    //     m_Shooter_D.setControl(velocityRequest.withVelocity(velocity));
    //     m_Shooter_U.setControl(velocityRequest.withVelocity(velocity));
    // }

    public void setMagicVelocity(double velocity, double accel) {
        m_Shooter_D.setControl(motionMagicVelRequest.withVelocity(velocity).withAcceleration(accel));
        m_Shooter_U.setControl(motionMagicVelRequest.withVelocity(velocity).withAcceleration(accel));
    }

    public void shoot_out() {
        setMagicVelocity(60, 300);
    }

    public void shoot_break() {
        setMagicVelocity(0, 300);
    }

    public boolean speed_ready() {
        return Math.abs(m_Shooter_D.getVelocity().getValueAsDouble() - 60) < 0.2;
    }

    // public void shoot_out_voltage() {
    //     setVoltage(10);
    // }

    // public void shoot_revert_voltage() {
    //     setVoltage(-5);
    // }

    // public void stop() {
    //     setVoltage(0);
    // }

}
