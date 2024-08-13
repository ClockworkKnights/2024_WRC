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

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    TalonFX m_Intake_L;
    TalonFX m_Intake_R;

    TalonFXConfiguration IntakeConfig_L;
    TalonFXConfiguration IntakeConfig_R;

    VelocityTorqueCurrentFOC velocityRequest;
    PositionVoltage positionRequest_L;
    PositionVoltage positionRequest_R;
    VoltageOut voltageRequest;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable intakeStats = inst.getTable("Intake");
    private final StringPublisher pub_note_state = intakeStats.getStringTopic("Note State").publish();

    public enum State {
        UNKNOWN,
        EATING,
        POS_WAIT_STOP,
        EATED_REVERSED,
        POPPING,
        POPPED,
    }

    public enum NoteState {
        UNKNOWN,
        FULL,
        EMPTY,
    }

    public double time_to_stop = 0;

    private State state;
    private NoteState note_state;

    public boolean auto_stop = false;

    public Intake() {
        m_Intake_L = new TalonFX(14, "canivore");
        m_Intake_R = new TalonFX(15, "canivore");

        // velocityRequest = new VelocityTorqueCurrentFOC(0);
        voltageRequest = new VoltageOut(0);
        positionRequest_L = new PositionVoltage(0);
        positionRequest_R = new PositionVoltage(0);

        configure();
        state = State.UNKNOWN;
        note_state = NoteState.UNKNOWN;
        pub_note_state.set("Unknown");
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
        if (state != State.EATING && Shooter.get_speed() < 10) {
            state = State.EATING;
            current_sample_index = 0;
            current_sample = new double[10];
            current_stable_A = 0;
            current_stable_B = 0;
            wait_current_unstable = false;
        }
        else if (Shooter.get_speed() >= 10) {
            note_state = NoteState.EMPTY;
            pub_note_state.set("No");
            current_sample_index = 0;
            current_sample = new double[10];
            current_stable_A = 0;
            current_stable_B = 0;
            wait_current_unstable = false;
        }
    }

    public void eat_volt(double volt) {
        setVoltage(volt);
        if (volt != 0) {
            state = volt > 0 ? State.EATING : State.POPPING;
        } else {
            state = State.UNKNOWN;
        }
    }

    public void eat_stop() {
        if (state == State.EATING) {
            reverse_once_pos(0);
        } else {
            stop();
        }
    }

    public void eat_out() {
        setVoltage(-5);
        if (state != State.POPPING) {
            state = State.POPPING;
            current_sample_index = 0;
            current_sample = new double[10];
            current_stable_A = 0;
            current_stable_B = 0;
            wait_current_unstable = false;
        }
    }

    public void reverse_once() {
        var positionNow_L = m_Intake_L.getPosition();
        var positionNow_R = m_Intake_R.getPosition();
        m_Intake_L.setControl(positionRequest_L.withPosition(positionNow_L.getValue() - 1));
        m_Intake_R.setControl(positionRequest_R.withPosition(positionNow_R.getValue() - 1));
        state = State.POS_WAIT_STOP;
        time_to_stop = Timer.getFPGATimestamp() + 0.3;
    }

    public void reverse_once_pos(double pos) {
        var positionNow_L = m_Intake_L.getPosition();
        var positionNow_R = m_Intake_R.getPosition();
        m_Intake_L.setControl(positionRequest_L.withPosition(positionNow_L.getValue() - pos));
        m_Intake_R.setControl(positionRequest_R.withPosition(positionNow_R.getValue() - pos));
        time_to_stop = Timer.getFPGATimestamp() + 0.3;
        state = State.POS_WAIT_STOP;
    }

    public State getState() {
        // System.out.println("Intake State: " + state);
        return state;
    }

    public void stop() {
        setVoltage(0);
    }

    private final int current_sample_size = 3;
    private double[] current_sample = new double[current_sample_size];
    private int current_sample_index = 0;
    private double current_stable_A = 0;
    private double current_stable_B = 0;
    private boolean wait_current_unstable = false;

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // System.out.println("Intake State: " + state);

        if (state == State.EATING) {
            current_sample[current_sample_index % current_sample_size] = m_Intake_L.getSupplyCurrent()
                    .getValueAsDouble();
            current_sample_index++;
            // current avg and std
            double current_avg = 0;
            double current_std = 0;
            for (int i = 0; i < current_sample_size; i++) {
                current_avg += current_sample[i];
            }
            current_avg /= current_sample_size;
            for (int i = 0; i < current_sample_size; i++) {
                current_std += Math.pow(current_sample[i] - current_avg, 2);
            }
            current_std = Math.sqrt(current_std / current_sample_size);
            if (current_sample_index > current_sample_size) {
                if (current_std < 0.1 && !wait_current_unstable) {
                    if (current_stable_A == 0) {
                        current_stable_A = current_avg;
                        wait_current_unstable = true;
                        System.out.println("Current A: " + current_stable_A);
                    }
                    else if (current_stable_A != 0) {
                        current_stable_B = current_avg;
                        wait_current_unstable = true;
                        if (current_stable_B - current_stable_A > 0.5) {
                            note_state= NoteState.FULL;
                            pub_note_state.set("Yes");
                            if (auto_stop) {
                                reverse_once_pos(0);
                            }
                        }
                        System.out.println("Current B: " + current_stable_B);
                    }
                }
                if (wait_current_unstable && current_std > 0.5) {
                    wait_current_unstable = false;
                    System.out.println("Current changing");
                }
            }
        }
        if(state == State.POPPING) {
            current_sample[current_sample_index % current_sample_size] = m_Intake_L.getStatorCurrent()
                    .getValueAsDouble();
            current_sample_index++;
            // current avg and std
            double current_avg = 0;
            double current_std = 0;
            for (int i = 0; i < current_sample_size; i++) {
                current_avg += current_sample[i];
            }
            current_avg /= current_sample_size;
            for (int i = 0; i < current_sample_size; i++) {
                current_std += Math.pow(current_sample[i] - current_avg, 2);
            }
            current_std = Math.sqrt(current_std / current_sample_size);
            if (current_sample_index > current_sample_size) {
                if (current_std < 0.1 && current_avg < 10 && current_avg > 3) {
                    note_state= NoteState.EMPTY;
                    pub_note_state.set("No");
                    if (auto_stop) {
                        stop();
                    }

                }
            }
        }

        if (state == State.POS_WAIT_STOP) {
            // System.out.println("Time to stop: " + time_to_stop + " Current time: " +
            // Timer.getFPGATimestamp());
            if (Timer.getFPGATimestamp() > time_to_stop) {
                stop();
                state = State.EATED_REVERSED;
            }
        }

    }

}
