package frc.robot.commands;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class NotePass extends Command {

    private final Shooter shooter;
    private final Arm arm;
    private final Intake intake;

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

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("NotePass");
    private final StringPublisher statePub = table.getStringTopic("State").publish();


    public NotePass() {
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
        // Stop everything
        shooter.stop();
        intake.stop();

        // Reverse the intake
        if (intake.getState() != Intake.State.POS_WAIT_STOP && intake.getState() != Intake.State.EATED_REVERSED) {
            intake.reverse_once_pos(1);
            state = State.INTAKE_REVERSE;
        }
        else {
            state = State.ARM_UP;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        statePub.set(state.toString());
        switch (state) {
            case INTAKE_REVERSE:
                if (intake.getState() == Intake.State.EATED_REVERSED) {
                    state = State.ARM_UP; 
                }
                break;
            case ARM_UP:
                if (arm.is_up()){
                    state = State.SHOOTER_SHOOT;
                    shooter.shoot_notepass();
                }
                break;
            case SHOOTER_SHOOT:
                shooter.shoot_notepass();
                if (shooter.speed_ready_notepass() && RobotController.getBatteryVoltage() > 10.5){
                    state = State.INTAKE_DELIVER;
                    intake.eat_volt(10);
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
                else {
                    shooter.shoot_notepass();
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
