// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelight_v1;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        // PortForwarding for Limelight during USB connection
        // for (int port = 5801; port <= 5809; port++) {
        // PortForwarder.add(port, "limelight.local", port);
        // }
        PortForwarder.add(5800, "limelight.local", 5800); 
        PortForwarder.add(5801, "limelight.local", 5801);         
        PortForwarder.add(5802, "10.64.87.17", 5800);
        PortForwarder.add(1181, "10.64.87.17", 1181);
        PortForwarder.add(1186, "10.64.87.17", 1186);


        FollowPathCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        Limelight_v1.vision_enabled = true;
        PhotonVision.vision_enabled = true;

        RobotContainer.drivetrain.aim_mode = Swerve.AimMode.NONE;
        RobotContainer.arm.stop();
        RobotContainer.shooter.stop();
        RobotContainer.intake.stop();
    }

    @Override
    public void disabledPeriodic() {
        if (DriverStation.getAlliance().isPresent())
            RobotContainer.isRedAlliance = DriverStation.getAlliance().get() == Alliance.Red;
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        Limelight_v1.vision_enabled = false;
        PhotonVision.vision_enabled = false;

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        Limelight_v1.vision_enabled = true;
        PhotonVision.vision_enabled = true;

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
