package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

public class Swerve extends SwerveDrivetrain implements Subsystem {

    public enum AimMode {
        NONE,
        SPEAKER,
        NOTE,
        AMP,
        PASS_NOTE,
    }

    public AimMode aim_mode = AimMode.NONE;

    // Setpoint updated by Aimer
    public double speaker_yaw_setpoint = 0.;
    public double note_pass_yaw_setpoint = 0.;
    public double amp_x_setpoint = 0.;
    public double amp_y_setpoint = 0.;
    public double amp_yaw_setpoint = 0.;

    public PIDController m_xController = new PIDController(0.1, 0, 0);
    public PIDController m_yController = new PIDController(0.1, 0, 0);
    public PIDController m_yawController = new PIDController(0.2, 0, 0.005);
    public double x_controller_output = 0;
    public double y_controller_output = 0;
    public double yaw_controller_output = 0;

    public static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // Max speed for close loop velocity
    public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private final NetworkTable debug_table = NetworkTableInstance.getDefault().getTable("Debug").getSubTable("Swerve");
    private final StringPublisher pub_aim_mode = debug_table.getSubTable("Aim").getStringTopic("Aim Mode").publish();
    private final DoublePublisher pub_yaw_error = debug_table.getSubTable("Aim").getDoubleTopic("Yaw Error").publish();
    private final DoublePublisher pub_yaw_output = debug_table.getSubTable("Aim").getDoubleTopic("Yaw Output").publish();
    private final StringPublisher pub_control_mode = debug_table.getSubTable("Control").getStringTopic("Mode").publish();
    private final DoublePublisher pub_control_x = debug_table.getSubTable("Control").getDoubleTopic("X").publish();
    private final DoublePublisher pub_control_y = debug_table.getSubTable("Control").getDoubleTopic("Y").publish();
    private final DoublePublisher pub_control_rot = debug_table.getSubTable("Control").getDoubleTopic("Rot").publish();

    public static final SwerveRequest.FieldCentric drive_FieldRelative = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic);
    public static final SwerveRequest.RobotCentric drive_RobotRelative = new SwerveRequest.RobotCentric()
            .withDeadband(0).withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagic);
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Rotation2d BlueHeadRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedHeadRotation = Rotation2d.fromDegrees(180);
    private boolean hasAppliedOperatorPerspective = false;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        System.err
                .println("RUNTIME WARNING: Swerve subsystem created without autobuilder called. This is not intended.");
    }

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPosition, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.005), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.005), // Rotation PID constants
                        4.25, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void resetPosition(Pose2d poseMeters) {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = this.getModule(i).getCachedPosition();
        }
        this.m_odometry.resetPosition(this.getPigeon2().getRotation2d(), modulePositions, poseMeters);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(drive_RobotRelative.withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    public Pose2d getPose() {
        var state = this.getState();
        return state.Pose;
    }

    public ChassisSpeeds getSpeeds() {
        return this.getState().speeds;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command cmdDriveFieldRelative(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot) {
        Supplier<Double> rot_output;
        Supplier<Double> x_output;
        Supplier<Double> y_output;
        rot_output = () -> {
            pub_control_mode.set("Field Relative " + aim_mode.toString());
            if (aim_mode == AimMode.SPEAKER || aim_mode == AimMode.NOTE || aim_mode == AimMode.AMP
                    || aim_mode == AimMode.PASS_NOTE) {
                if (yaw_controller_output > MaxAngularRate) {
                    pub_control_rot.set(MaxAngularRate);
                    return MaxAngularRate;
                } else if (yaw_controller_output < -MaxAngularRate) {
                    pub_control_rot.set(-MaxAngularRate);
                    return -MaxAngularRate;
                }
                pub_control_rot.set(yaw_controller_output);
                return yaw_controller_output;
            } else {
                if (rot.get() > MaxAngularRate) {
                    pub_control_rot.set(MaxAngularRate);
                    return MaxAngularRate;
                } else if (rot.get() < -MaxAngularRate) {
                    pub_control_rot.set(-MaxAngularRate);
                    return -MaxAngularRate;
                }
                pub_control_rot.set(rot.get());
                return rot.get();
            }
        };
        x_output = () -> {
            pub_control_mode.set("Field Relative " + aim_mode.toString());
            if (aim_mode == AimMode.AMP) {
                if (x_controller_output > MaxSpeed) {
                    pub_control_x.set(MaxSpeed);
                    return MaxSpeed;
                } else if (x_controller_output < -MaxSpeed) {
                    pub_control_x.set(-MaxSpeed);
                    return -MaxSpeed;
                }
                pub_control_x.set(x_controller_output);
                return x_controller_output;
            } else {
                if (x.get() > MaxSpeed) {
                    pub_control_x.set(MaxSpeed);
                    return MaxSpeed;
                } else if (x.get() < -MaxSpeed) {
                    pub_control_x.set(-MaxSpeed);
                    return -MaxSpeed;
                }
                pub_control_x.set(x.get());
                return x.get();
            }
        };
        y_output = () -> {
            pub_control_mode.set("Field Relative " + aim_mode.toString());
            if (aim_mode == AimMode.AMP) {
                if (y_controller_output > MaxSpeed) {
                    pub_control_y.set(MaxSpeed);
                    return MaxSpeed;
                } else if (y_controller_output < -MaxSpeed) {
                    pub_control_y.set(-MaxSpeed);
                    return -MaxSpeed;
                }
                pub_control_y.set(y_controller_output);
                return y_controller_output;
            } else {
                if (y.get() > MaxSpeed) {
                    pub_control_y.set(MaxSpeed);
                    return MaxSpeed;
                } else if (y.get() < -MaxSpeed) {
                    pub_control_y.set(-MaxSpeed);
                    return -MaxSpeed;
                }
                pub_control_y.set(y.get());
                return y.get();
            }
        };
        Supplier<Double> rot_deadband = () -> {
            if (aim_mode == AimMode.SPEAKER || aim_mode == AimMode.NOTE || aim_mode == AimMode.AMP
                    || aim_mode == AimMode.PASS_NOTE) {
                return 0.;
            } else {
                return 0.1 * MaxAngularRate;
            }
        };
        return applyRequest(() -> drive_FieldRelative.withVelocityX(x_output.get()).withVelocityY(y_output.get())
                .withRotationalRate(rot_output.get()).withRotationalDeadband(rot_deadband.get()));
    }

    public Command cmdBrakeX() {
        return applyRequest(() -> brake);
    }

    public Command cmdPointWheels(Supplier<Double> x, Supplier<Double> y) {
        return applyRequest(() -> point.withModuleDirection(new Rotation2d(x.get(), y.get())));
    }

    public void resetHeading() {
        DriverStation.getAlliance().ifPresent((allianceColor) -> {
            if (allianceColor == Alliance.Red) {
                this.getPigeon2().setYaw(RedHeadRotation.getDegrees());
                this.setOperatorPerspectiveForward(RedHeadRotation);
                this.seedFieldRelative(new Pose2d(this.getPose().getX(), this.getPose().getY(),
                        RedHeadRotation));
                this.getPigeon2().setYaw(RedHeadRotation.getDegrees());
                this.setOperatorPerspectiveForward(RedHeadRotation);
                this.seedFieldRelative(new Pose2d(this.getPose().getX(), this.getPose().getY(),
                        RedHeadRotation));
            } else {
                this.getPigeon2().setYaw(BlueHeadRotation.getDegrees());
                this.setOperatorPerspectiveForward(BlueHeadRotation);
                this.seedFieldRelative(new Pose2d(this.getPose().getX(), this.getPose().getY(),
                        BlueHeadRotation));
                this.getPigeon2().setYaw(BlueHeadRotation.getDegrees());
                this.setOperatorPerspectiveForward(BlueHeadRotation);
                this.seedFieldRelative(new Pose2d(this.getPose().getX(), this.getPose().getY(),
                        BlueHeadRotation));
            }
        });
        this.seedFieldRelative();
    }

    private void updatePIDControllers() {
        double yaw_error = 0;

        if (aim_mode == AimMode.SPEAKER) {
            yaw_error = this.speaker_yaw_setpoint - this.getPose().getRotation().getDegrees();
        } else if (aim_mode == AimMode.NOTE) {
            yaw_error = PhotonVision.note_detection_yaw_error;
            yaw_error = Math.max(-90, Math.min(90, yaw_error));
        } else if (aim_mode == AimMode.AMP) {
            yaw_error = 90 - this.getPose().getRotation().getDegrees();
        } else if (aim_mode == AimMode.PASS_NOTE) {
            yaw_error = note_pass_yaw_setpoint - this.getPose().getRotation().getDegrees();
        }

        while (yaw_error > 180) {
            yaw_error -= 360;
        }
        while (yaw_error < -180) {
            yaw_error += 360;
        }

        this.yaw_controller_output = -m_yawController.calculate(yaw_error, 0);
        pub_yaw_output.set(yaw_controller_output);
        pub_yaw_error.set(yaw_error);

        double x_error = 0;
        double y_error = 0;

        if (aim_mode == AimMode.AMP) {
            x_error = amp_x_setpoint - this.getPose().getTranslation().getX();
            y_error = amp_y_setpoint - this.getPose().getTranslation().getY();
        }

        this.x_controller_output = m_xController.calculate(x_error, 0);
        this.y_controller_output = m_yController.calculate(y_error, 0);
    }

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedHeadRotation : BlueHeadRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        pub_aim_mode.set(aim_mode.toString());

        updatePIDControllers();
    }
}
