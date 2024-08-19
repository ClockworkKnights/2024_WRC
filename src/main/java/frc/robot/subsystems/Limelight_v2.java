package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Limelight_v2 extends SubsystemBase {

    public static boolean vision_enabled = true;

    Swerve m_swerve = TunerConstants.DriveTrain;
    Pigeon2 m_gyro = m_swerve.getPigeon2();
    
    private final DoubleArrayPublisher limelightPub = NetworkTableInstance.getDefault()
            .getTable("Pose").getDoubleArrayTopic("limelight")
            .publish();

    public Limelight_v2() {
    }

    @Override
    public void periodic() {
        boolean doRejectUpdate = !vision_enabled;

        LimelightHelpers.SetRobotOrientation("limelight", -m_swerve.getPigeon2().getAngle(),
                -m_swerve.getPigeon2().getRate(), 0, 0,
                0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (Math.abs(m_gyro.getRate()) > 720) {
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            var stddev = VecBuilder.fill(.7, .7, 999);
            if (mt2.tagCount == 1) {
                stddev = VecBuilder.fill(5, 5, 999999);
            }
            if (mt2.tagCount == 2) {
                stddev = VecBuilder.fill(1, 1, 9999);
            }
            m_swerve.setVisionMeasurementStdDevs(stddev);
            m_swerve.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
            limelightPub.set(new double[] {
                    mt2.pose.getX(),
                    mt2.pose.getY(),
                    mt2.pose.getRotation().getDegrees()
            });
        }
    }

    public static Pose2d getLimelightPose() {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        return mt2.pose;
    }
}
