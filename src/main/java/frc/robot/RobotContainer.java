// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public static boolean isRedAlliance = false;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandPS5Controller joystick = new CommandPS5Controller(0); // My joystick
  private final Swerve drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake intake = new Intake(); // My intake
  private final Arm arm = new Arm();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity) // I want field-centric
                                                       // driving in open loop
      .withSteerRequestType(SteerRequestType.MotionMagic);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * (isRedAlliance ? -1 : 1)) // Drive
                                                                                                                      // forward
                                                                                                                      // with
            // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * (isRedAlliance ? -1 : 1)) // Drive left with negative X
                                                                                       // (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.circle().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    // joystick.L3().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    
    // Intake in
    joystick.R2().onTrue(Commands.runOnce(() -> intake.eat_in()));
    joystick.R2().onFalse(Commands.runOnce(() -> intake.stop()));
    // Intake out
    joystick.R1().onTrue(Commands.runOnce(() -> intake.eat_out()));
    joystick.R1().onFalse(Commands.runOnce(() -> intake.stop()));

    // Arm Up, Arm Down
    joystick.povUp().onTrue(Commands.runOnce(() -> arm.arm_up_volt(false)));
    joystick.povUp().onFalse(Commands.runOnce(() -> arm.stop()));
    joystick.povDown().onTrue(Commands.runOnce(() -> arm.arm_down_volt(false)));
    joystick.povDown().onFalse(Commands.runOnce(() -> arm.stop()));

    joystick.L1().onTrue(Commands.runOnce(() -> arm.arm_up()));
    joystick.L2().onTrue(Commands.runOnce(() -> arm.arm_down()));
    
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
