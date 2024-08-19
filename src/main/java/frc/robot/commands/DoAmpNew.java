package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.AimMode;

public class DoAmpNew extends Command {

    private final Shooter shooter;
    private final Arm arm;
    private final Intake intake;
    private final Swerve swerve;

    public static boolean enable_aim = false;

    private enum State {
        GOTO_FAR_CHECKPOINT,
        GOTO_WALL,
        TO_STAGE_1,
        TO_STAGE_2,
        TO_STAGE_3,
        ARM_SHUAI,
        ARM_DOWN,
        FINISHED,
    }

    private final Pose3d BlueAllianceAmpFar = new Pose3d(72.5 * 0.0254, 323.00 * 0.0254, 53.38 * 0.0254, null);
    private final Pose3d RedAllianceAmpFar = new Pose3d(578.77 * 0.0254, 323.00 * 0.0254, 53.38 * 0.0254, null);

    private Pose3d AmpTarget = BlueAllianceAmpFar;

    private Timer timer = new Timer();

    private State state = State.FINISHED;

    public DoAmpNew() {
        // Use addRequirements() here to declare subsystem dependencies.
        shooter = RobotContainer.shooter;
        arm = RobotContainer.arm;
        intake = RobotContainer.intake;
        swerve = RobotContainer.drivetrain;
        // addRequirements(shooter, arm, intake);
        state = State.FINISHED;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Stop all subsystems
        shooter.stop();
        intake.stop();
        arm.stop();
        state = State.GOTO_FAR_CHECKPOINT;
        // Set target
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty() || !enable_aim) {
            // Empty alliance, no drivetrain movement
            raise_arm_prep();
            shooter_shoot();
            state = State.TO_STAGE_1;
            return;
        } else {
            AmpTarget = alliance.get() == DriverStation.Alliance.Red ? RedAllianceAmpFar : BlueAllianceAmpFar;
        }
        // Too far, abort
        var pose = swerve.getPose();
        if (pose.getTranslation().getDistance(AmpTarget.getTranslation().toTranslation2d()) > 3) {
            state = State.FINISHED;
            return;
        }
    }

    private final double ARM_STAGE_1 = 18;
    // private final double SHOOT_ANGLE = 9;
    private final double ARM_STAGE_2 = 24;
    private final double SHOOTER_SPEED = 11;
    private final double SHOOTER_ACCEL = 600;

    private void update_swerve_far_checkpoint() {
        RobotContainer.drivetrain.aim_mode = AimMode.AMP;
        RobotContainer.drivetrain.amp_x_setpoint = AmpTarget.getTranslation().getX();
        RobotContainer.drivetrain.amp_y_setpoint = AmpTarget.getTranslation().getY() - 1;
        RobotContainer.drivetrain.amp_yaw_setpoint = Math.PI/2;
    }

    private boolean at_far_checkpoint() {
        var pose = swerve.getPose();
        return Math.abs(pose.getTranslation().getY() - AmpTarget.getTranslation().getY() + 1) < 0.1
        && Math.abs(pose.getTranslation().getX() - AmpTarget.getTranslation().getX()) < 0.05
        && Math.abs(pose.getRotation().getDegrees() - 90) < 4;
    }

    private void update_swerve_goto_wall() {
        RobotContainer.drivetrain.aim_mode = AimMode.AMP;
        RobotContainer.drivetrain.amp_x_setpoint = 0.5;
        RobotContainer.drivetrain.amp_y_setpoint = 0.5;
        RobotContainer.drivetrain.amp_yaw_setpoint = 0;
    }

    private boolean at_wall() {
        return true;
    }

    private void raise_arm_prep() {
        arm.arm_pos_magic(ARM_STAGE_1, 100, 300, 900);
    }

    private void raise_arm_swing() {
        arm.arm_pos_magic(ARM_STAGE_2, 125, 600, 3000);
    }

    private void shooter_shoot() {
        shooter.shoot_magic_vel(SHOOTER_SPEED, SHOOTER_ACCEL);
    }

    @Override
    public void execute() {
        switch (state) {
            case GOTO_FAR_CHECKPOINT:
                update_swerve_far_checkpoint();
                if (at_far_checkpoint()) {
                    state = State.GOTO_WALL;
                }
            case GOTO_WALL:
                update_swerve_goto_wall();
                if (at_wall()) {
                    raise_arm_prep();
                    shooter_shoot();
                    state = State.TO_STAGE_1;
                }
            case TO_STAGE_1:
                if (arm.get_angle() > ARM_STAGE_1 - 3) {
                    intake.eat_in();
                }
                if (arm.get_angle() > ARM_STAGE_1 - 1) {
                    state = State.TO_STAGE_2;
                    raise_arm_swing();
                }
                break;
            case TO_STAGE_2:
                if (arm.get_angle() > ARM_STAGE_2 - 1) {
                    state = State.TO_STAGE_3;
                }
                break;
            case TO_STAGE_3:
                if (timer.get() > 1) {
                    state = State.ARM_DOWN;
                    shooter.shoot_break();
                    intake.stop();
                    arm.arm_down();
                }
                break;
            case ARM_DOWN:
                state = State.FINISHED;
                break;
            case FINISHED:
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        arm.arm_down();
        intake.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return state == State.FINISHED || state == State.ARM_DOWN;
    }
}
