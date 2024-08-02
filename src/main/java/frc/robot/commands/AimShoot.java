package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AimShoot extends Command {

    private final Shooter shooter;
    private final Arm arm;
    private final Intake intake;

    private double intakeL_startPos;

    private enum State {
        INTAKE_REVERSE,
        ARM_UP,
        SHOOTER_SHOOT,
        INTAKE_DELIVER,
        ARM_DOWN,
        FINISHED,
    }

    private Timer timer = new Timer();

    private State state = State.FINISHED;

    public AimShoot() {
        // Use addRequirements() here to declare subsystem dependencies.
        shooter = new Shooter();
        arm = new Arm();
        intake = new Intake();
        addRequirements(shooter, arm, intake);
        state = State.FINISHED;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.arm_up();
        shooter.shoot_break();
        intake.stop();
        intakeL_startPos = intake.getPosition_L();
        if (intake.getState() != Intake.State.REVERSED){
            intake.reverse_once();
            state = State.INTAKE_REVERSE;
        }
        else {
            state = State.ARM_UP;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (state) {
            case INTAKE_REVERSE:
                if (arm.is_up() && intakeL_startPos - intake.getPosition_L() > 0.5) {
                    state = State.SHOOTER_SHOOT;
                    shooter.shoot_out();
                }
                break;
            case ARM_UP:
                if (arm.is_up()){
                    state = State.SHOOTER_SHOOT;
                    shooter.shoot_out();
                }
                break;
            case SHOOTER_SHOOT:
                if (shooter.speed_ready()){
                    state = State.INTAKE_DELIVER;
                    intake.eat_in();
                    timer.reset();
                    timer.start();
                }
                break;
            case INTAKE_DELIVER:
                if (timer.get() > 0.5){
                    state = State.ARM_DOWN;
                    shooter.shoot_break();
                    arm.arm_down();
                    intake.stop();
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
        shooter.shoot_break();
        arm.arm_down();
        intake.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return state == State.FINISHED || state == State.ARM_DOWN;
    }
}
