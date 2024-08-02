package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {
    TalonFX m_Intake_L;
    TalonFX m_Intake_R;

    TalonFXConfiguration IntakeConfig_L;
    TalonFXConfiguration IntakeConfig_R;

    VelocityTorqueCurrentFOC velocityRequest;
    PositionVoltage positionRequest_L;
    PositionVoltage positionRequest_R;
    VoltageOut voltageRequest;

    public enum State {
        UNKNOWN,
        EATED,
        REVERSED,
        POPPED,
    }

    private State state;

    public Intake() {
        m_Intake_L = new TalonFX(14, "canivore");
        m_Intake_R = new TalonFX(15, "canivore");

        // velocityRequest = new VelocityTorqueCurrentFOC(0);
        voltageRequest = new VoltageOut(0);
        positionRequest_L = new PositionVoltage(0);
        positionRequest_R = new PositionVoltage(0);

        configure();
        state = State.UNKNOWN;
    }

    public void configure() {
        IntakeConfig_L = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(20)
                                .withSupplyCurrentLimitEnable(true))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast))
                .withSlot0(new Slot0Configs()
                        .withKP(3)
                        .withKI(0)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0)
                        .withKG(0))
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                        .withVoltageOpenLoopRampPeriod(0.3))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(0.06));
        IntakeConfig_R = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(20)
                                .withSupplyCurrentLimitEnable(true))
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast))
                .withSlot0(new Slot0Configs()
                        .withKP(3)
                        .withKI(0)
                        .withKD(0)
                        .withKS(0)
                        .withKV(0)
                        .withKA(0)
                        .withKG(0))
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                        .withVoltageOpenLoopRampPeriod(0.3))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(0.06));

        m_Intake_L.getConfigurator().apply(IntakeConfig_L);
        m_Intake_R.getConfigurator().apply(IntakeConfig_R);

    }

    public void setVoltage(double voltage) {
        m_Intake_L.setControl(voltageRequest.withOutput(voltage));
        m_Intake_R.setControl(voltageRequest.withOutput(voltage));
    }

    public double getPosition_L() {
        return m_Intake_L.getPosition().getValueAsDouble();
    }

    public double getPosition_R() {
        return m_Intake_R.getPosition().getValueAsDouble();
    }

    // public void setVelocity(double velocity) {
    // m_Intake_L.setControl(velocityRequest.withVelocity(velocity));
    // m_Intake_R.setControl(velocityRequest.withVelocity(velocity));
    // }

    public void eat_in() {
        setVoltage(5);
        state = State.EATED;
    }

    public void eat_out() {
        setVoltage(-5);
        state = State.POPPED;
    }

    public void reverse_once() {
        var positionNow_L = m_Intake_L.getPosition();
        var positionNow_R = m_Intake_R.getPosition();
        m_Intake_L.setControl(positionRequest_L.withPosition(positionNow_L.getValue() - 0.8));
        m_Intake_R.setControl(positionRequest_R.withPosition(positionNow_R.getValue() - 0.8));
        state = State.REVERSED;
    }

    public State getState() {
        return state;
    }

    public void stop() {
        setVoltage(0);
    }

}
