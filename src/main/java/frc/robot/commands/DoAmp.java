package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DoAmp extends Command {

    private final Shooter shooter;
    private final Arm arm;
    private final Intake intake;

    private enum State {
        ARM_UP,
        SHOOTER_SHOOT,
        WAIT_ARM,
        ARM_SHUAI,
        ARM_DOWN,
        FINISHED,
    }

    private Timer timer = new Timer();

    private State state = State.FINISHED;

    public DoAmp() {
        // Use addRequirements() here to declare subsystem dependencies.
        shooter = RobotContainer.shooter;
        arm = RobotContainer.arm;
        intake = RobotContainer.intake;
        addRequirements(shooter, arm, intake);
        state = State.FINISHED;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.arm_pos_magic(ARM_STAGE_1, 100, 300, 900);
        shooter.shoot_break();
        intake.stop();
        if (intake.getState() != Intake.State.EATED_REVERSED) {
            intake.reverse_once();
        }
        state = State.ARM_UP;
    }

    private final double ARM_STAGE_1 = 12;
    private final double SHOOT_ANGLE = 9;
    private final double ARM_STAGE_2 = 24;
    private final double SHOOTER_SPEED = 11;
    private final double SHOOTER_ACCEL = 600;

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (state) {
            case ARM_UP:
                if (arm.get_angle() > ARM_STAGE_1 - 3) {
                    state = State.SHOOTER_SHOOT;
                    shooter.shoot_magic_vel(SHOOTER_SPEED, SHOOTER_ACCEL);
                    arm.arm_pos_magic(ARM_STAGE_2, 120, 600, 3000);
                    intake.eat_in();
                }
                break;
            case SHOOTER_SHOOT:
                if (shooter.speed_ready(SHOOTER_SPEED)) {
                    state = State.WAIT_ARM;
                    // timer.reset();
                    // timer.start();
                }
                break;
            case WAIT_ARM:
                if (arm.get_angle() > SHOOT_ANGLE) {
                    intake.eat_in();
                    state = State.ARM_SHUAI;
                }
                break;
            case ARM_SHUAI:
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
