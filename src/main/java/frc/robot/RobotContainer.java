// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.AimMode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight_v1;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Aimer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AimShoot;
import frc.robot.commands.DoAmp;
import frc.robot.commands.NotePass;

public class RobotContainer {

    public static boolean isRedAlliance = false;

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final CommandPS5Controller joystick = new CommandPS5Controller(0); // My joystick
    public static final Swerve drivetrain = TunerConstants.DriveTrain;
    public static final Intake intake = new Intake();
    public static final Arm arm = new Arm();
    public static final Shooter shooter = new Shooter();
    public static final Limelight_v1 limelight = new Limelight_v1();
    public static final Aimer aimer = new Aimer();
    public static final PhotonVision photonVision = new PhotonVision();
    public static final Climber climber = new Climber();

    private final SendableChooser<Command> autoChooser;
    private final Field2d field = new Field2d();

    private Command AimShoot = new AimShoot();
    private Command DoAmp = new DoAmp();
    private Command NotePass = new NotePass();

    private final Telemetry logger = new Telemetry(Swerve.MaxSpeed);

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.cmd_drive(
                        () -> (-joystick.getLeftY() * Swerve.MaxSpeed * (isRedAlliance ? -1 : 1)),
                        () -> (-joystick.getLeftX() * Swerve.MaxSpeed * (isRedAlliance ? -1 : 1)),
                        () -> (-joystick.getRightX() * Swerve.MaxAngularRate)));

        // joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.circle().whileTrue(drivetrain
        // .applyRequest(() -> point.withModuleDirection(new
        // Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // reset the field-centric heading on left bumper press
        joystick.L3().onTrue(drivetrain.runOnce(() -> {
            drivetrain.setHead();
        }));

        // Intake in
        joystick.L2().onTrue(Commands.runOnce(() -> {
            intake.eat_in();
            intake.auto_stop = false;
        }));
        joystick.L2().onFalse(Commands.runOnce(() -> {
            intake.eat_stop();
            intake.auto_stop = false;
        }));
        // Intake out
        joystick.L1().onTrue(Commands.runOnce(() -> intake.eat_out()));
        joystick.L1().onFalse(Commands.runOnce(() -> intake.stop()));

        // Arm Up, Arm Down
        joystick.povUp().onTrue(Commands.runOnce(() -> arm.arm_up_volt(false)));
        joystick.povUp().onFalse(Commands.runOnce(() -> arm.stop()));
        joystick.povDown().onTrue(Commands.runOnce(() -> arm.arm_down_volt(false)));
        joystick.povDown().onFalse(Commands.runOnce(() -> arm.stop()));

        joystick.R2().whileTrue(Commands.run(() -> {
            drivetrain.aim_mode = AimMode.SPEAKER;
            arm.arm_up_autoaim();
            Limelight_v1.vision_yaw = true;
            PhotonVision.vision_yaw = true;
        }));
        joystick.R2().onFalse(Commands.runOnce(() -> {
            arm.arm_down();
            drivetrain.aim_mode = AimMode.NONE;
            Limelight_v1.vision_yaw = false;
            PhotonVision.vision_yaw = false;
        }));
        joystick.R1().whileTrue(AimShoot);

        // joystick.triangle().whileTrue(Commands.runOnce(() -> shooter.shoot_amp()));
        // joystick.triangle().onFalse(Commands.runOnce(() -> shooter.shoot_break()));
        // joystick.circle().whileTrue(Commands.runOnce(() -> arm.arm_amp()));
        // joystick.cross().onTrue(Commands.runOnce(() -> intake.reverse_once()));

        joystick.circle().whileTrue(Commands.run(() -> {
            drivetrain.aim_mode = AimMode.NOTE;
        }));
        joystick.circle().onFalse(Commands.run(() -> {
            drivetrain.aim_mode = AimMode.NONE;
        }));
        joystick.triangle().whileTrue(DoAmp);

        joystick.cross().onTrue(Commands.run(() -> {
            drivetrain.aim_mode = AimMode.PASS_NOTE;
            arm.arm_up_notepass();
        }));
        joystick.cross().whileTrue(NotePass);
        joystick.cross().onFalse(Commands.run(() -> {
            arm.arm_down();
            shooter.shoot_break();
            drivetrain.aim_mode = AimMode.NONE;
        }));

        // joystick.square().onTrue(Commands.runOnce(() -> climber.climb_up()));
        // joystick.cross().onTrue(Commands.runOnce(() -> climber.climb_down()));
        // joystick.square().onFalse(Commands.runOnce(() -> climber.stop()));
        // joystick.cross().onFalse(Commands.runOnce(() -> climber.stop()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {

        NamedCommands.registerCommand("arm_preset_preload", Commands.runOnce(() -> arm.setPosition(9)));
        NamedCommands.registerCommand("shooter_preset_preload", Commands.runOnce(() -> shooter.shoot_out(55)));
        NamedCommands.registerCommand("eat_in", Commands.runOnce(() -> intake.eat_volt(9)));
        NamedCommands.registerCommand("eat_stop", Commands.runOnce(() -> intake.stop()));
        NamedCommands.registerCommand("arm_preset_c1", Commands.runOnce(() -> arm.setPosition(8.7)));
        NamedCommands.registerCommand("arm_preset_c2", Commands.runOnce(() -> arm.setPosition(6.8)));
        NamedCommands.registerCommand("shooter_stop", Commands.runOnce(() -> shooter.shoot_break()));
        NamedCommands.registerCommand("arm_stop", Commands.runOnce(() -> arm.stop()));
        NamedCommands.registerCommand("eat_reverse_once", Commands.runOnce(() -> intake.reverse_once_pos(0)));
        NamedCommands.registerCommand("shooter_preset_c2", Commands.runOnce(() -> shooter.shoot_out(78)));

        // NamedCommands.registerCommand("arm_preset_preload", Commands.runOnce(() -> {
        // }));
        // NamedCommands.registerCommand("shooter_preset_preload", Commands.runOnce(()
        // -> {
        // }));
        // NamedCommands.registerCommand("eat_in", Commands.runOnce(() -> {
        // }));
        // NamedCommands.registerCommand("eat_stop", Commands.runOnce(() -> {
        // }));
        // NamedCommands.registerCommand("arm_preset_c1", Commands.runOnce(() -> {
        // }));
        // NamedCommands.registerCommand("arm_preset_c2", Commands.runOnce(() -> {
        // }));
        // NamedCommands.registerCommand("shooter_stop", Commands.runOnce(() -> {
        // }));
        // NamedCommands.registerCommand("arm_stop", Commands.runOnce(() -> {
        // }));
        // NamedCommands.registerCommand("eat_reverse_once", Commands.runOnce(() -> {
        // }));
        // NamedCommands.registerCommand("shooter_preset_c2", Commands.runOnce(() -> {
        // }));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Field", field);
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        // return Commands.print("No autonomous command configured");
    }
}
