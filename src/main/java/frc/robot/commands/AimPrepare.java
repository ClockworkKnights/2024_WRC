package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight_v1;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.AimMode;

public class AimPrepare extends Command {

    private final Shooter shooter;
    private final Arm arm;
    private final Swerve swerve;

    public AimPrepare() {
        // Use addRequirements() here to declare subsystem dependencies.
        shooter = RobotContainer.shooter;
        arm = RobotContainer.arm;
        swerve = RobotContainer.drivetrain;
        // addRequirements(shooter, arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerve.aim_mode = AimMode.SPEAKER;
        shooter.setMagicVelocity(Shooter.shooter_speaker_target, 140);
        arm.arm_up_speaker();
        Limelight_v1.yaw_correction_enabled = true;
        PhotonVision.yaw_correction_enabled = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerve.aim_mode = AimMode.SPEAKER;
        if (!AimShoot.running) {
            shooter.setMagicVelocity(Shooter.shooter_speaker_target, 140);
            arm.arm_up_speaker();
        }
        Limelight_v1.yaw_correction_enabled = true;
        PhotonVision.yaw_correction_enabled = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        arm.arm_down();
        swerve.aim_mode = AimMode.NONE;
        Limelight_v1.yaw_correction_enabled = false;
        PhotonVision.yaw_correction_enabled = false;
    }
}
