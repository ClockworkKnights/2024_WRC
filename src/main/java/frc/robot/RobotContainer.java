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
import frc.robot.commands.AimPrepare;
import frc.robot.commands.AimShoot;
import frc.robot.commands.DoAmp;
import frc.robot.commands.NotePass;

public class RobotContainer {

    public static boolean isRedAlliance = false;

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

    private Command AimPrepare = new AimPrepare();
    private Command AimShoot = new AimShoot();
    private Command DoAmp = new DoAmp();
    private Command NotePass = new NotePass();

    private final Telemetry logger = new Telemetry(Swerve.MaxSpeed);

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.cmdDriveFieldRelative(
                        () -> (-joystick.getLeftY() * Swerve.MaxSpeed * (isRedAlliance ? -1 : 1)),
                        () -> (-joystick.getLeftX() * Swerve.MaxSpeed * (isRedAlliance ? -1 : 1)),
                        () -> (-joystick.getRightX() * Swerve.MaxAngularRate)));

        joystick.L3().onTrue(drivetrain.runOnce(() -> {
            drivetrain.resetHeading();
        }));

        // L1 L2 - Intake
        joystick.L2().onTrue(Commands.runOnce(() -> {
            intake.eat_in();
        }));
        joystick.L2().onFalse(Commands.runOnce(() -> {
            intake.eat_stop();
        }));
        joystick.L1().onTrue(Commands.runOnce(() -> intake.eat_out()));
        joystick.L1().onFalse(Commands.runOnce(() -> intake.stop()));

        // R1 R2 - AimSpeaker, Shooter
        joystick.R2().whileTrue(AimPrepare);
        joystick.R1().whileTrue(AimShoot);

        // Button Right - AimNote
        joystick.circle().whileTrue(Commands.run(() -> {
            // drivetrain.cmdDriveFieldRelative(() -> 0., () -> 0., () -> 0.).execute();
            drivetrain.aim_mode = AimMode.NOTE;
        }));
        joystick.circle().onFalse(Commands.run(() -> {
            drivetrain.aim_mode = AimMode.NONE;
        }));
        // Button Up - Amp
        joystick.triangle().whileTrue(DoAmp);
        // Button Down - NotePass
        joystick.cross().whileTrue(NotePass);

        // DEBUG - Manually Arm Up, Arm Down
        // joystick.povUp().onTrue(Commands.runOnce(() -> arm.arm_up_volt(false)));
        // joystick.povUp().onFalse(Commands.runOnce(() -> arm.stop()));
        // joystick.povDown().onTrue(Commands.runOnce(() -> arm.arm_down_volt(false)));
        // joystick.povDown().onFalse(Commands.runOnce(() -> arm.stop()));

        // DEBUG - Swerve Brake
        // joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.circle().whileTrue(drivetrain
        // .applyRequest(() -> point.withModuleDirection(new
        // Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // DEBUG - Climber
        // joystick.square().onTrue(Commands.runOnce(() -> climber.climb_up()));
        // joystick.cross().onTrue(Commands.runOnce(() -> climber.climb_down()));
        // joystick.square().onFalse(Commands.runOnce(() -> climber.stop()));
        // joystick.cross().onFalse(Commands.runOnce(() -> climber.stop()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {

        NamedCommands.registerCommand("arm_preset_preload", Commands.runOnce(() -> arm.setPosition(9.3)));
        NamedCommands.registerCommand("shooter_preset_preload", Commands.runOnce(() -> shooter.shoot_out(55)));
        NamedCommands.registerCommand("eat_in", Commands.runOnce(() -> intake.eat_volt(9)));
        NamedCommands.registerCommand("eat_stop", Commands.runOnce(() -> intake.stop()));
        NamedCommands.registerCommand("arm_preset_c1", Commands.runOnce(() -> arm.setPosition(8.7)));
        NamedCommands.registerCommand("arm_preset_c2", Commands.runOnce(() -> arm.setPosition(6.5)));
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
