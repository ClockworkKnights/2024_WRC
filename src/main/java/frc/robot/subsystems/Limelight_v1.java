package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Limelight_v1 extends SubsystemBase {

    public static boolean vision_yaw = false;

    Swerve m_swerve = TunerConstants.DriveTrain;
    Pigeon2 m_gyro = new Pigeon2(20, "canivore");

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher limelightPub = table.getDoubleArrayTopic("limelight").publish();

    private Pose2d m_lastPose = new Pose2d();

    private static boolean first_pose = true;

    public Limelight_v1() {
    }

    @Override
    public void periodic() {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (mt1.tagCount == 0) {
            doRejectUpdate = true;
        }
        var pose = mt1.pose;
        var timestamp = mt1.timestampSeconds;
        if (pose.getX() < 0 || pose.getX() > 16.8 || pose.getY() < 0 || pose.getY() > 8.4) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            double dev_dist = Math.exp(1.5716 * mt1.rawFiducials[0].distToCamera) * 0.0774;
            double dev_amb = Math.exp(7.2861 * mt1.rawFiducials[0].ambiguity) * 0.5818;
            double dev = Math.min(dev_dist, dev_amb);
            double dev_yaw = dev;
            if (first_pose) {
                dev = 0.001;
                dev_yaw = 9;
                first_pose = false;
            }
            if (vision_yaw) {
                m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(dev, dev, dev_yaw));
                m_swerve.addVisionMeasurement(
                        new Pose2d(pose.getX(), pose.getY(),
                                // m_swerve.getPigeon2().getRotation2d()),
                                pose.getRotation()),
                        timestamp);
            } else {
                dev_yaw = 99999999.;
                m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(dev, dev, dev_yaw));
                m_swerve.addVisionMeasurement(
                        new Pose2d(pose.getX(), pose.getY(),
                                m_swerve.getPigeon2().getRotation2d()),
                        // pose.getRotation()),
                        timestamp);
            }
            this.m_lastPose = pose;
        }

        limelightPub.set(new double[] {
                m_lastPose.getX(),
                m_lastPose.getY(),
                m_lastPose.getRotation().getDegrees()
        });

    }

    public Pose2d getLimelightPose() {
        return m_lastPose;
    }
}
