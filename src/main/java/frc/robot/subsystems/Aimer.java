package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Aimer extends SubsystemBase {

    private static final Limelight_v2 limelight = new Limelight_v2();
    // private static final PhotonVision photonVision = new PhotonVision();

    Pose3d targetSpeaker = new Pose3d();
    Pose3d targetAmp = new Pose3d();
    Rotation2d targetSpeakerHeading = new Rotation2d();
    Rotation2d targetAmpHeading = new Rotation2d();

    private final Pose3d BlueAllianceSpeaker = new Pose3d(-1.50 * 0.0254 + 0.15, 218.42 * 0.0254, 2.05, null);
    private final Pose3d RedAllianceSpeaker = new Pose3d(652.73 * 0.0254 - 0.15, 218.42 * 0.0254,  2.05, null);
    private final Pose3d BlueAllianceAmp = new Pose3d(72.5 * 0.0254, 323.00 * 0.0254, 53.38 * 0.0254, null);
    private final Pose3d RedAllianceAmp = new Pose3d(578.77 * 0.0254, 323.00 * 0.0254, 53.38 * 0.0254, null);

    private final Rotation2d AmpHeading = new Rotation2d(90. / 180. * Math.PI);
    private final Rotation2d BlueSpeakerHeading = new Rotation2d(180. / 180. * Math.PI);
    private final Rotation2d RedSpeakerHeading = new Rotation2d(0. / 180. * Math.PI);

    private boolean hasAppliedOperatorPerspective = false;

    private final Swerve m_swerve = TunerConstants.DriveTrain;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Aimer");
    private final DoubleArrayPublisher speakerPub = table.getDoubleArrayTopic("speaker_target").publish();
    private final DoubleArrayPublisher shooterPub = table.getDoubleArrayTopic("Shooter").publish();

    private final NetworkTable table_arm = table.getSubTable("Arm");
    private final DoublePublisher arm_target_pub = table_arm.getDoubleTopic("arm_target").publish();
    private final DoublePublisher arm_now_pub = table_arm.getDoubleTopic("arm_now").publish();
    private final DoublePublisher arm_error_pub = table_arm.getDoubleTopic("arm_error").publish();
    private final DoublePublisher arm_output = table_arm.getDoubleTopic("arm_output").publish();


    public Aimer() {
        updateAlliance();
    }

    public void updateAlliance() {
        DriverStation.getAlliance().ifPresent((allianceColor) -> {
            targetSpeaker = allianceColor == Alliance.Red ? RedAllianceSpeaker : BlueAllianceSpeaker;
            targetAmp = allianceColor == Alliance.Red ? RedAllianceAmp : BlueAllianceAmp;
            targetSpeakerHeading = allianceColor == Alliance.Red ? RedSpeakerHeading : BlueSpeakerHeading;
            targetAmpHeading = AmpHeading;
            hasAppliedOperatorPerspective = true;
        });
    }

    public void update_speaker() {
        double target_x = targetSpeaker.getTranslation().getX();
        double target_y = targetSpeaker.getTranslation().getY();

        double swerve_x = m_swerve.getState().Pose.getX();
        double swerve_y = m_swerve.getState().Pose.getY();
        double serve_vel_x = m_swerve.getState().speeds.vxMetersPerSecond;  // m/s
        double serve_vel_y = m_swerve.getState().speeds.vyMetersPerSecond;  // m/s

        // Shooter is back 0.5m from the center of the robot
        double shooter_x = swerve_x - 0.2 * Math.cos(m_swerve.getState().Pose.getRotation().getRadians());
        double shooter_y = swerve_y - 0.2 * Math.sin(m_swerve.getState().Pose.getRotation().getRadians());

        shooterPub.set(new double[] {
            shooter_x,
            shooter_y,
            m_swerve.getState().Pose.getRotation().getDegrees()
        });

        double distance = Math.sqrt(Math.pow(target_x - swerve_x, 2) + Math.pow(target_y - swerve_y, 2));
        double note_speed = 10; // TODO: m/s
        double note_time = distance / note_speed;
        double note_dx = note_time * serve_vel_x;
        double note_dy = note_time * serve_vel_y;
        double target_x_new = target_x + note_dx;
        double target_y_new = target_y + note_dy;
        double target_height = targetSpeaker.getTranslation().getZ();

        double dist_angle_compensation = 2.545 * distance - 7.50; // 0.1m height compensation

        final double arm_90deg_rot = 21.8; // 21.8 sensor rotation for 90 deg arm rotation

        speakerPub.set(new double[] {
            target_x_new,
            target_y_new,
            target_height
        });

        double target_heading = Math.atan2(target_y_new - swerve_y, target_x_new - swerve_x);   
        Rotation2d target_heading_rot = new Rotation2d(target_heading);
        m_swerve.yaw_setpoint = target_heading_rot.getDegrees();        // -180~180, 0 points to right, right hand coordinate

        double arm_angle = Math.atan2(target_height, Math.sqrt(Math.pow(target_x_new - swerve_x, 2) + Math.pow(target_y_new - swerve_y, 2)));
        double arm_angle_deg = arm_angle * 180 / Math.PI + dist_angle_compensation;
        double arm_sensor_target = arm_angle_deg * arm_90deg_rot / 90;
        Arm.arm_autoaim_target = arm_sensor_target;

        double shooter_target = 40 + 6 * distance; // 40m/s + 0.5m/s/m * distance
        Shooter.shooter_autoaim_target = shooter_target;

        arm_target_pub.set(arm_angle_deg);
        
    }

    @Override
    public void periodic() {
        // Update alliance if we are disabled or haven't applied the operator perspective
        // Ensure the target is updated to the correct alliance
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            updateAlliance();
        }
        // Get Pose
        Pose2d pose = m_swerve.getState().Pose;
        update_speaker();
    }
}
