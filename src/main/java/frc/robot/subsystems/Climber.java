package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    public static TalonFX m_Climber_L;
    public static TalonFX m_Climber_R;

    public static TalonFXConfiguration ClimberConfig_L;
    public static TalonFXConfiguration ClimberConfig_R;

    public Climber() {
        m_Climber_L = new TalonFX(18, "canivore");
        m_Climber_R = new TalonFX(19, "canivore");
    }

    public void climb_up() {
        m_Climber_L.set(0.5);
        m_Climber_R.set(0.5);
    }

    public void climb_down() {
        m_Climber_L.set(-0.2);
        m_Climber_R.set(-0.2);
    }

    public void stop() {
        m_Climber_L.set(0);
        m_Climber_R.set(0);
    }
}
