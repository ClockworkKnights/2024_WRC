package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.AimMode;

public class NotePass extends Command {

    private final Shooter shooter;
    private final Arm arm;
    private final Intake intake;
    private final Swerve drivetrain;
    private final CommandPS5Controller joystick;


    private enum State {
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
        drivetrain = RobotContainer.drivetrain;
        joystick = RobotContainer.joystick;
        // addRequirements(shooter, arm, intake);
        state = State.FINISHED;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Stop everything
        shooter.stop();
        intake.stop();
        drivetrain.aim_mode = AimMode.PASS_NOTE;
        arm.arm_up_notepass();
        state = State.ARM_UP;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        statePub.set(state.toString());
        // drivetrain.cmdDriveFieldRelative(
        //         () -> (-joystick.getLeftY() * Swerve.MaxSpeed * (RobotContainer.isRedAlliance ? -1 : 1)),
        //         () -> (-joystick.getLeftX() * Swerve.MaxSpeed * (RobotContainer.isRedAlliance ? -1 : 1)),
        //         () -> (-joystick.getRightX() * Swerve.MaxAngularRate)
        //         ).execute();
        switch (state) {
            case ARM_UP:
                arm.arm_up_notepass();
                double deg_now = drivetrain.getPose().getRotation().getDegrees();
                double deg_target = drivetrain.note_pass_yaw_setpoint;
                double deg_error = deg_target - deg_now;
                while (deg_error > 180) {
                    deg_error -= 360;
                }
                while (deg_error < -180) {
                    deg_error += 360;
                }
                if (arm.is_up() && Math.abs(deg_error) < 5) {
                    state = State.SHOOTER_SHOOT;
                    shooter.shoot_notepass();
                }
                break;
            case SHOOTER_SHOOT:
                shooter.shoot_notepass();
                arm.arm_up_notepass();
                if (shooter.speed_ready_notepass() && RobotController.getBatteryVoltage() > 10.5) {
                    state = State.INTAKE_DELIVER;
                    intake.eat_volt(10);
                    timer.reset();
                    timer.start();
                }
                break;
            case INTAKE_DELIVER:
                if (timer.get() > 0.5) {
                    state = State.FINISHED;
                    shooter.shoot_break();
                    arm.arm_down();
                    intake.stop();
                } else {
                    arm.arm_up_notepass();
                    shooter.shoot_notepass();
                }
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
        drivetrain.aim_mode = AimMode.NONE;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return state == State.FINISHED || state == State.ARM_DOWN;
    }
}
