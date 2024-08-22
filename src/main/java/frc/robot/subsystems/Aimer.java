package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Aimer extends SubsystemBase {

    Pose3d targetSpeaker = new Pose3d();
    Pose3d targetAmp = new Pose3d();
    Pose3d targetNotePassFall = new Pose3d();

    private final Pose3d BlueAllianceSpeaker = new Pose3d(-1.50 * 0.0254, 218.42 * 0.0254, 2.05, null);
    private final Pose3d RedAllianceSpeaker = new Pose3d(652.73 * 0.0254, 218.42 * 0.0254, 2.05, null);
    private final Pose3d BlueNotePassFall = new Pose3d(72.5 * 0.0254, 323.00 * 0.0254 - 1.5, 53.38 * 0.0254, null);
    private final Pose3d RedNotePassFall = new Pose3d(578.77 * 0.0254, 323.00 * 0.0254 - 1.5, 53.38 * 0.0254, null);
    private final Pose3d BlueAllianceAmp = new Pose3d(72.5 * 0.0254, 323.00 * 0.0254, 53.38 * 0.0254, null);
    private final Pose3d RedAllianceAmp = new Pose3d(578.77 * 0.0254, 323.00 * 0.0254, 53.38 * 0.0254, null);

    private boolean hasAppliedOperatorPerspective = false;

    private final Swerve m_swerve = TunerConstants.DriveTrain;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable tableSpeaker = inst.getTable("Debug").getSubTable("Aimer").getSubTable("Speaker");
    private final DoubleArrayPublisher speakerPub = tableSpeaker.getDoubleArrayTopic("Target").publish();
    private final DoubleArrayPublisher shooterPub = tableSpeaker.getDoubleArrayTopic("Robot Shooter Pose").publish();
    private final DoublePublisher speakerDistPub = tableSpeaker.getDoubleTopic("Distance").publish();
    private final DoublePublisher arm_target_pub = tableSpeaker.getDoubleTopic("Arm Angle Target").publish();
    private final DoublePublisher shooter_target_pub = tableSpeaker.getDoubleTopic("Shooter Target").publish();

    private final NetworkTable tableNotePass = inst.getTable("Debug").getSubTable("Aimer").getSubTable("NotePass");
    private final DoubleArrayPublisher notepassTargetPub = tableNotePass.getDoubleArrayTopic("Target").publish();
    private final DoublePublisher notepassArmTargetDegPub = tableNotePass.getDoubleTopic("Arm Angle Target (deg)")
            .publish();
    private final DoublePublisher notepassShooterTargetPub = tableNotePass.getDoubleTopic("Shooter Target").publish();
    private final DoublePublisher notepassYawPub = tableNotePass.getDoubleTopic("Yaw Target").publish();

    private final ShuffleboardTab sbTab = Shuffleboard.getTab("Aimer");
    private final GenericEntry sbDebugSpeakerAimer = sbTab.add("Speaker Debug Mode", false)
            .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private final GenericEntry sbSpeakerAngleTheo = sbTab.add("Speaker Angle Theo", 0).getEntry();
    private final GenericEntry sbSpeakerAngleInput = sbTab.add("Speaker Angle Set", 0).getEntry();
    private final GenericEntry sbSpeakerAngleOffset = sbTab.add("Speaker Ang Offset Set", 0).getEntry();
    private final GenericEntry sbSpeakerSpeedTheo = sbTab.add("Speaker Speed Theo", 0).getEntry();
    private final GenericEntry sbSpeakerDistTheo = sbTab.add("Speaker Dist Theo", 0).getEntry();

    private final double gravity_g = 9.8;

    public Aimer() {
        updateAlliance();
    }

    public void updateAlliance() {
        DriverStation.getAlliance().ifPresent((allianceColor) -> {
            targetSpeaker = allianceColor == Alliance.Red ? RedAllianceSpeaker : BlueAllianceSpeaker;
            targetAmp = allianceColor == Alliance.Red ? RedAllianceAmp : BlueAllianceAmp;
            targetNotePassFall = allianceColor == Alliance.Red ? RedNotePassFall : BlueNotePassFall;
            hasAppliedOperatorPerspective = true;
        });
    }

    public void update_speaker() {
        double target_x = targetSpeaker.getTranslation().getX();
        double target_y = targetSpeaker.getTranslation().getY();

        double swerve_x = m_swerve.getPose().getX();
        double swerve_y = m_swerve.getPose().getY();
        double swerve_vel_x = m_swerve.getState().speeds.vxMetersPerSecond; // m/s
        double swerve_vel_y = m_swerve.getState().speeds.vyMetersPerSecond; // m/s

        // Shooter is back 0.5m from the center of the robot
        double shooter_x = swerve_x - 0.2 *
                Math.cos(m_swerve.getPose().getRotation().getRadians());
        double shooter_y = swerve_y - 0.2 *
                Math.sin(m_swerve.getPose().getRotation().getRadians());

        shooterPub.set(new double[] {
                shooter_x,
                shooter_y,
                m_swerve.getPose().getRotation().getDegrees()
        });

        double distance = Math.sqrt(Math.pow(target_x - swerve_x, 2) +
                Math.pow(target_y - swerve_y, 2));
        sbSpeakerDistTheo.setDouble(distance);
        double note_speed = 10; // TODO: m/s
        double note_time = distance / note_speed;
        double note_dx = note_time * swerve_vel_x;
        double note_dy = note_time * swerve_vel_y;
        double target_x_new = target_x + note_dx;
        double target_y_new = target_y + note_dy;
        double target_height = targetSpeaker.getTranslation().getZ();

        final double arm_90deg_rot = 21.8;
        // 21.8 sensor rotation for 90 deg arm rotation

        speakerPub.set(new double[] {
                target_x_new,
                target_y_new,
                target_height
        });

        double target_heading = Math.atan2(target_y_new - swerve_y, target_x_new -
                swerve_x);
        Rotation2d target_heading_rot = new Rotation2d(target_heading);
        m_swerve.speaker_yaw_setpoint = target_heading_rot.getDegrees();
        // -180~180, 0 points to right, right hand coordinate

        double dist_angle_compensation = 0; // 2.545 * distance - 7.50;
        // 0.1m height compensation
        double target_height_compensation = 0;
        if (distance < 2) {
            target_height_compensation = 0.15 * distance - 0.3;
        } else if (distance > 3.5) {
            target_height_compensation = 0.4 * (distance - 3.5);
        }

        double shooter_target = 40 + 8 * distance; // 40m/s + 0.5m/s/m * distance

        // double arm_angle = Math.atan2(target_height - 0.15 +
        //         target_height_compensation,
        //         Math.sqrt(Math.pow(target_x_new - swerve_x, 2) + Math.pow(target_y_new -
        //                 swerve_y, 2)));
        // double arm_angle_deg = arm_angle * 180 / Math.PI + dist_angle_compensation;
        // // if (shooter_target > 90) {
        // //     arm_angle_deg += (shooter_target - 90) / 8;
        // // }
        // double arm_sensor_target = arm_angle_deg * arm_90deg_rot / 90;
        double x = distance;
        // 8/19
        // double arm_sensor_target = -0.013233 *x*x*x*x + 0.065266 *x*x*x + 0.717571 *x*x - 6.275255 *x + 20.049852;
        // 8/22 make it higher
        double arm_sensor_target = 0.396076 *x*x - 4.384524 *x + 18.385368;

        sbSpeakerAngleTheo.setDouble(arm_sensor_target);
        arm_target_pub.set(arm_sensor_target);
        if (sbDebugSpeakerAimer.getBoolean(false) && sbSpeakerAngleInput.getDouble(0) > 0) {
            arm_sensor_target = sbSpeakerAngleInput.getDouble(arm_sensor_target);
        }
        if (!sbDebugSpeakerAimer.getBoolean(false)) {
            arm_sensor_target += sbSpeakerAngleOffset.getDouble(0);
        }
        Arm.arm_speaker_target = arm_sensor_target;
        // double x = distance;
        // double arm_sensor_target = 0.00910581 *x*x*x*x - 0.23298615 *x*x*x +
        // 2.17574395 *x*x - 9.45186811 *x + 22.19559391;


        if (shooter_target > 90) {
            shooter_target = 90;
        }
        speakerDistPub.set(distance);
        Shooter.shooter_speaker_target = shooter_target;
        sbSpeakerSpeedTheo.setDouble(shooter_target);
        shooter_target_pub.set(shooter_target);
    }

    public void update_speaker_parabola() {

        double target_x = targetSpeaker.getTranslation().getX();
        double target_y = targetSpeaker.getTranslation().getY();
        double target_z = targetSpeaker.getTranslation().getZ();

        double swerve_x = m_swerve.getPose().getX();
        double swerve_y = m_swerve.getPose().getY();
        double swerve_vel_x = m_swerve.getState().speeds.vxMetersPerSecond; // m/s
        double swerve_vel_y = m_swerve.getState().speeds.vyMetersPerSecond; // m/s
        // Shooter is back 0.5m from the center of the robot
        double shooter_x = swerve_x - 0.2 *
                Math.cos(m_swerve.getPose().getRotation().getRadians());
        double shooter_y = swerve_y - 0.2 *
                Math.sin(m_swerve.getPose().getRotation().getRadians());

        // Solve parabola
        final double shooter_perimeter = 0.045 * Math.PI; // TODO: Update perimeter
        double distance = Math.sqrt(Math.pow(target_x - shooter_x, 2) +
                Math.pow(target_y - shooter_y, 2));
        double top_d = distance;
        double v0 = getParabola_byTop_InitSpeed(target_z, top_d);
        double theta = getParabola_byTop_InitAngle(target_z, top_d);
        double v0_rps = v0 / shooter_perimeter;
        if (v0_rps > 95) {
            v0_rps = 95;
        }

        // Movement shooting
        double v0_approach = v0 * Math.cos(theta);
        double v0_lift = v0 * Math.sin(theta);
        // v1: speed of robot drivetrain. approach: speed to the target. normal: side
        // speed
        double yaw_target = Math.atan2(target_y - swerve_y, target_x - swerve_x);
        double yaw_v1 = Math.atan2(swerve_vel_y, swerve_vel_x);
        double v1_approach = Math.cos(yaw_v1 - yaw_target)
                * Math.sqrt(swerve_vel_x * swerve_vel_x + swerve_vel_y * swerve_vel_y);
        double v1_normal = -Math.sin(yaw_v1 - yaw_target)
                * Math.sqrt(swerve_vel_x * swerve_vel_x + swerve_vel_y * swerve_vel_y);
        double v0v1_approach = v0_approach + v1_approach;
        double v0_new = Math.sqrt(v0v1_approach * v0v1_approach + v0_lift * v0_lift);
        double theta_new = Math.atan2(v0_lift, v0v1_approach);
        v0_new = Math.max(0, Math.min(80 * shooter_perimeter, v0_new));
        double v0_rps_new = v0_new / shooter_perimeter * 1.03;
        theta_new = Math.max(0, Math.min(Math.PI / 2, theta_new));

        // Apply target - shooter
        Shooter.shooter_speaker_target = v0_rps_new;
        // Apply target - arm
        double theta_deg = theta_new * 180 / Math.PI;
        final double arm_90deg_rot = 21.8;
        // 21.8 sensor rotation for 90 deg arm rotation
        Arm.arm_speaker_target = theta_deg * arm_90deg_rot / 90;

        double flying_time = 2 * v0_lift / gravity_g;
        double normal_distance = v1_normal * flying_time;

        double target_x_new = target_x + normal_distance * Math.sin(yaw_target);
        double target_y_new = target_y + normal_distance * Math.cos(yaw_target);

        speakerDistPub.set(distance);

        // Apply target - swerve
        double target_heading = Math.atan2(target_y_new - swerve_y, target_x_new -
                swerve_x);
        Rotation2d target_heading_rot = new Rotation2d(target_heading);
        m_swerve.speaker_yaw_setpoint = target_heading_rot.getDegrees();

        arm_target_pub.set(theta_deg);
        shooter_target_pub.set(v0_rps_new);
        speakerPub.set(new double[] {
                target_x_new,
                target_y_new,
                target_z,
        });

    }

    public double getParabola_byTop_InitSpeed(double top_h, double top_d) {
        // High point h, projection distance d
        // Return initial speed (and angle in another function)
        double g = gravity_g;
        // double theta = Math.atan2(2 * top_h, top_d);
        double v = Math.sqrt(2 * g * (top_d * top_d + 16 * top_h * top_h) / (16 * top_h));
        return v;
    }

    public double getParabola_byTop_InitAngle(double top_h, double top_d) {
        // High point h, projection distance d
        // Return initial angle (and speed in another function)
        // double g = gravity_g;
        double theta = Math.atan2(2 * top_h, top_d);
        // double v = Math.sqrt(2 * g * (top_d * top_d + 16 * top_h * top_h) / (16 *
        // top_h));
        return theta;
    }

    public Optional<Double> getParabola_bySpeedDistance_InitAngle(double v0, double top_d) {
        // Initial speed v, projection distance d
        // Return high point (and angle in another function)
        double g = gravity_g;
        if (v0 < 0 || top_d < 0) {
            return Optional.empty();
        }
        double theta = 1 / 2 * Math.asin(2 * top_d * g / (v0 * v0));
        if (Math.abs(2 * top_d * g / (v0 * v0)) > 1) {
            return Optional.empty();
        }
        return Optional.of(theta);
    }

    public void update_pass_note() {

        double target_x = targetNotePassFall.getTranslation().getX();
        double target_y = targetNotePassFall.getTranslation().getY();

        double swerve_x = m_swerve.getPose().getX();
        double swerve_y = m_swerve.getPose().getY();
        double swerve_vel_x = m_swerve.getState().speeds.vxMetersPerSecond; // m/s
        double swerve_vel_y = m_swerve.getState().speeds.vyMetersPerSecond; // m/s
        // Shooter is back 0.5m from the center of the robot
        double shooter_x = swerve_x - 0.2 * Math.cos(m_swerve.getPose().getRotation().getRadians());
        double shooter_y = swerve_y - 0.2 * Math.sin(m_swerve.getPose().getRotation().getRadians());

        // Solve parabola
        final double shooter_perimeter = 0.045 * Math.PI; // TODO: Update perimeter
        double double_distance = Math.sqrt(Math.pow(target_x - shooter_x, 2) + Math.pow(target_y - shooter_y, 2));
        double top_d = double_distance / 2;
        double v0 = getParabola_byTop_InitSpeed(3.7, top_d);
        double theta = getParabola_byTop_InitAngle(3.7, top_d);
        double v0_rps = v0 / shooter_perimeter;
        if (v0_rps > 80) {
            v0_rps = 80;
            v0 = v0_rps * shooter_perimeter;
            var try_theta = getParabola_bySpeedDistance_InitAngle(v0, top_d);
            if (try_theta.isPresent()) {
                theta = try_theta.get();
            }
        }

        // Movement shooting
        double v0_approach = v0 * Math.cos(theta);
        double v0_lift = v0 * Math.sin(theta);
        // v1: speed of robot drivetrain. approach: speed to the target. normal: side
        // speed
        double yaw_target = Math.atan2(target_y - swerve_y, target_x - swerve_x);
        double yaw_v1 = Math.atan2(swerve_vel_y, swerve_vel_x);
        double v1_approach = Math.cos(yaw_v1 - yaw_target)
                * Math.sqrt(swerve_vel_x * swerve_vel_x + swerve_vel_y * swerve_vel_y);
        double v1_normal = -Math.sin(yaw_v1 - yaw_target)
                * Math.sqrt(swerve_vel_x * swerve_vel_x + swerve_vel_y * swerve_vel_y);
        double v0v1_approach = v0_approach + v1_approach;
        double v0_new = Math.sqrt(v0v1_approach * v0v1_approach + v0_lift * v0_lift);
        double theta_new = Math.atan2(v0_lift, v0v1_approach);
        v0_new = Math.max(0, Math.min(80 * shooter_perimeter, v0_new));
        double v0_rps_new = v0_new / shooter_perimeter * 1.03;
        theta_new = Math.max(0, Math.min(Math.PI / 2, theta_new));

        // Apply target - shooter
        Shooter.shooter_notepass_target = v0_rps_new;
        // Apply target - arm
        double theta_deg = theta_new * 180 / Math.PI;
        final double arm_90deg_rot = 21.8; // 21.8 sensor rotation for 90 deg arm rotation
        Arm.arm_notepass_target = theta_deg * arm_90deg_rot / 90;

        double flying_time = 2 * v0_lift / gravity_g;
        double normal_distance = v1_normal * flying_time;

        double target_x_new = target_x + normal_distance * Math.sin(yaw_target);
        double target_y_new = target_y + normal_distance * Math.cos(yaw_target);

        notepassTargetPub.set(new double[] {
                target_x_new,
                target_y_new,
        });

        // Apply target - swerve
        double target_heading = Math.atan2(target_y_new - swerve_y, target_x_new - swerve_x);
        Rotation2d target_heading_rot = new Rotation2d(target_heading);
        m_swerve.note_pass_yaw_setpoint = target_heading_rot.getDegrees();

        notepassArmTargetDegPub.set(theta_deg);
        notepassShooterTargetPub.set(v0_rps_new);
        notepassYawPub.set(target_heading_rot.getDegrees());

    }

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            updateAlliance();
        }
        // update_speaker_parabola();
        update_speaker();
        update_pass_note();
    }
}
