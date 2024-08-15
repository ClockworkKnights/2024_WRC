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

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {

    public enum AimMode {
        NONE,
        SPEAKER,
        NOTE,
        AMP,
        PASS_NOTE,
    }

    public AimMode aim_mode = AimMode.NONE;
    public double speaker_yaw_setpoint = 0.;
    public double note_pass_yaw_setpoint = 0.;
    public double yaw_controller_output = 0;
    public PIDController m_yawController = new PIDController(0.2, 0, 0.005);

    public double x_setpoint = 0.;
    public double y_setpoint = 0.;
    public PIDController m_xController = new PIDController(0.1, 0, 0);
    public PIDController m_yController = new PIDController(0.1, 0, 0);
    public double x_controller_output = 0;
    public double y_controller_output = 0;

    public static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Swerve");
    private final StringPublisher pub_aim_mode = table.getSubTable("Aim").getStringTopic("Aim Mode").publish();
    private final DoublePublisher pub_yaw_error = table.getSubTable("Aim").getDoubleTopic("Yaw Error").publish();
    private final DoublePublisher pub_yaw_output = table.getSubTable("Aim").getDoubleTopic("Yaw Output").publish();

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity) // I want field-centric
                                                             // driving in open loop
            .withSteerRequestType(SteerRequestType.MotionMagic);
    public static final SwerveRequest.RobotCentric drive_RobotRelative = new SwerveRequest.RobotCentric()
            .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity) // I want robot-centric
                                                             // driving in open loop
            .withSteerRequestType(SteerRequestType.MotionMagic);
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    public void resetPosition(Pose2d poseMeters) {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = this.getModule(i).getCachedPosition();
        }

        // System.out.println("Reset Position: " + poseMeters.getX() + " " +
        // poseMeters.getY() + " "
        // + poseMeters.getRotation().getDegrees());
        this.m_odometry.resetPosition(this.getPigeon2().getRotation2d(), modulePositions, poseMeters);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(drive_RobotRelative.withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
        // System.out.println("Drive Robot Relative: " + speeds.vxMetersPerSecond + " "
        // + speeds.vyMetersPerSecond + " "
        // + speeds.omegaRadiansPerSecond);
    }

    public Pose2d getPose() {
        // System.out.println("Pose get: " + this.getPose().getX() + " " +
        // this.getPose().getY() + " "
        // + this.getPose().getRotation().getDegrees());
        var state = this.getState();
        return state.Pose;
    }

    public ChassisSpeeds getSpeeds() {
        // System.out.println("Speeds get: " + this.getState().speeds.vxMetersPerSecond
        // + " "
        // + this.getState().speeds.vyMetersPerSecond + " " +
        // this.getState().speeds.omegaRadiansPerSecond);
        return this.getState().speeds;
    }

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        // AutoBuilder.configureHolonomic(
        // this::getPose, // Robot pose supplier
        // this::resetPosition, // Method to reset odometry (will be called if your auto
        // has a starting pose)
        // this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        // this::driveRobotRelative, // Method that will drive the robot given ROBOT
        // RELATIVE ChassisSpeeds
        // new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
        // likely live in your
        // // Constants class
        // new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        // new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        // 4.5, // Max module speed, in m/s
        // 0.4, // Drive base radius in meters. Distance from robot center to furthest
        // module.
        // new ReplanningConfig() // Default path replanning config. See the API for the
        // options here
        // ),
        // () -> {
        // // Boolean supplier that controls when the path will be mirrored for the red
        // // alliance
        // // This will flip the path being followed to the red side of the field.
        // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent()) {
        // return alliance.get() == DriverStation.Alliance.Red;
        // }
        // return false;
        // },
        // this // Reference to this subsystem to set requirements
        // );
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

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command cmd_drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot) {
        Supplier<Double> rot_output;
        Supplier<Double> x_output;
        Supplier<Double> y_output;
        rot_output = () -> {
            if (aim_mode == AimMode.SPEAKER || aim_mode == AimMode.NOTE || aim_mode == AimMode.AMP
                    || aim_mode == AimMode.PASS_NOTE) {
                if (yaw_controller_output > MaxAngularRate) {
                    return MaxAngularRate;
                } else if (yaw_controller_output < -MaxAngularRate) {
                    return -MaxAngularRate;
                }
                return yaw_controller_output;
            } else {
                if (rot.get() > MaxAngularRate) {
                    return MaxAngularRate;
                } else if (rot.get() < -MaxAngularRate) {
                    return -MaxAngularRate;
                }
                return rot.get();
            }
        };
        x_output = () -> {
            if (aim_mode == AimMode.AMP) {
                if (x_controller_output > MaxSpeed) {
                    return MaxSpeed;
                } else if (x_controller_output < -MaxSpeed) {
                    return -MaxSpeed;
                }
                return x_controller_output;
            } else {
                if (x.get() > MaxSpeed) {
                    return MaxSpeed;
                } else if (x.get() < -MaxSpeed) {
                    return -MaxSpeed;
                }
                return x.get();
            }
        };
        y_output = () -> {
            if (aim_mode == AimMode.AMP) {
                if (y_controller_output > MaxSpeed) {
                    return MaxSpeed;
                } else if (y_controller_output < -MaxSpeed) {
                    return -MaxSpeed;
                }
                return y_controller_output;
            } else {
                if (y.get() > MaxSpeed) {
                    return MaxSpeed;
                } else if (y.get() < -MaxSpeed) {
                    return -MaxSpeed;
                }
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
        return applyRequest(() -> drive.withVelocityX(x_output.get()).withVelocityY(y_output.get())
                .withRotationalRate(rot_output.get()).withRotationalDeadband(rot_deadband.get()));
    }

    public Command cmd_brake_x() {
        return applyRequest(() -> brake);
    }

    public Command cmd_pointwheels(Supplier<Double> x, Supplier<Double> y) {
        return applyRequest(() -> point.withModuleDirection(new Rotation2d(x.get(), y.get())));
    }

    public void setHead() {
        DriverStation.getAlliance().ifPresent((allianceColor) -> {
            if (allianceColor == Alliance.Red) {
                this.getPigeon2().setYaw(RedAlliancePerspectiveRotation.getDegrees());
                this.setOperatorPerspectiveForward(RedAlliancePerspectiveRotation);
                this.seedFieldRelative(new Pose2d(this.getPose().getX(), this.getPose().getY(),
                        RedAlliancePerspectiveRotation));
                this.getPigeon2().setYaw(RedAlliancePerspectiveRotation.getDegrees());
                this.setOperatorPerspectiveForward(RedAlliancePerspectiveRotation);
                this.seedFieldRelative(new Pose2d(this.getPose().getX(), this.getPose().getY(),
                        RedAlliancePerspectiveRotation));
            } else {
                this.getPigeon2().setYaw(BlueAlliancePerspectiveRotation.getDegrees());
                this.setOperatorPerspectiveForward(BlueAlliancePerspectiveRotation);
                this.seedFieldRelative(new Pose2d(this.getPose().getX(), this.getPose().getY(),
                        BlueAlliancePerspectiveRotation));
                this.getPigeon2().setYaw(BlueAlliancePerspectiveRotation.getDegrees());
                this.setOperatorPerspectiveForward(BlueAlliancePerspectiveRotation);
                this.seedFieldRelative(new Pose2d(this.getPose().getX(), this.getPose().getY(),
                        BlueAlliancePerspectiveRotation));
            }
        });
        this.seedFieldRelative();
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        pub_aim_mode.set(aim_mode.toString());

        double yaw_error = 0;
        if (aim_mode == AimMode.SPEAKER) {

            yaw_error = -this.speaker_yaw_setpoint + this.getPose().getRotation().getDegrees();
            if (yaw_error > 180) {
                yaw_error -= 360;
            }
            if (yaw_error < -180) {
                yaw_error += 360;
            }
            this.yaw_controller_output = m_yawController.calculate(yaw_error, 0);
        } else if (aim_mode == AimMode.NOTE) {
            yaw_error = -PhotonVision.yaw_error;
            if (yaw_error > 90) {
                yaw_error = 90;
            }
            if (yaw_error < -90) {
                yaw_error = -90;
            }
            this.yaw_controller_output = m_yawController.calculate(yaw_error / 10, 0);
        } else if (aim_mode == AimMode.AMP) {
            yaw_error = -90 + this.getPose().getRotation().getDegrees();
            if (yaw_error > 180) {
                yaw_error -= 360;
            }
            if (yaw_error < -180) {
                yaw_error += 360;
            }
            this.yaw_controller_output = m_yawController.calculate(yaw_error, 0);
        } else if (aim_mode == AimMode.PASS_NOTE) {
            yaw_error = 0;
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    yaw_error = -160 + this.getPose().getRotation().getDegrees();
                } else {
                    yaw_error = -20 + this.getPose().getRotation().getDegrees();
                }
            }

            if (yaw_error > 180) {
                yaw_error -= 360;
            }
            if (yaw_error < -180) {
                yaw_error += 360;
            }
            this.yaw_controller_output = m_yawController.calculate(yaw_error, 0);
        }

        pub_yaw_output.set(this.yaw_controller_output);
        pub_yaw_error.set(yaw_error);

        this.x_controller_output = m_xController.calculate(this.getPose().getTranslation().getX(),
                this.x_setpoint);
        this.y_controller_output = m_yController.calculate(this.getPose().getTranslation().getY(),
                this.y_setpoint);

        // yawtargetpub.set(this.yaw_setpoint);
        // yawnowpub.set(this.getPose().getRotation().getDegrees());
        // yawerrorpub.set(yaw_error);
        // yawoutput.set(this.yaw_controller_output);
    }
}
