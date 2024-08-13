package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class PhotonVision extends SubsystemBase  {

    Swerve m_swerve = TunerConstants.DriveTrain;
    Pigeon2 m_gyro = new Pigeon2(20, "canivore");

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher graypub = table.getDoubleArrayTopic("photonvision_gray").publish();
    private final DoubleArrayPublisher colorpub = table.getDoubleArrayTopic("photonvision_color").publish();

    PhotonPoseEstimator pose_estimator_gray;
    PhotonPoseEstimator pose_estimator_color;

    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public PhotonVision() {
        // Forward Camera
        var cam_gray = new PhotonCamera("gray");
        Transform3d robotToCam_gray = new Transform3d(new Translation3d(0., 0., 0.), new Rotation3d(0, 0, 0));
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.

        var cam_color = new PhotonCamera("color");
        Transform3d robotToCam_color = new Transform3d(new Translation3d(0., 0., 0.), new Rotation3d(0, 0, 0));

        // Construct PhotonPoseEstimator
        pose_estimator_gray = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam_gray, robotToCam_gray);
        pose_estimator_color = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam_color, robotToCam_color);
    }

    @Override
    public void periodic() {
        // boolean doRejectUpdate = false;
        // EstimatedRobotPose pose_gray = null;
        // EstimatedRobotPose pose_color = null;
        // try {
        //     pose_gray = pose_estimator_gray.update().get();
        // } catch (Exception e) {
        // }
        // try {
        //     pose_color = pose_estimator_color.update().get();
        // } catch (Exception e) {
        // }

        // m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));

        // if (Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
        //                                       // vision updates
        // {
        //     doRejectUpdate = true;
        // }

        // if (!doRejectUpdate) {
        //     if (pose_gray != null) {
        //         m_swerve.addVisionMeasurement(
        //                 pose_gray.estimatedPose.toPose2d(),
        //                 pose_gray.timestampSeconds);
        //         graypub.set(new double[] {
        //             pose_gray.estimatedPose.getTranslation().getX(),
        //             pose_gray.estimatedPose.getTranslation().getY(),
        //             pose_gray.estimatedPose.getRotation().toRotation2d().getDegrees()
        //         });
        //     }
        //     if (pose_color != null) {
        //         m_swerve.addVisionMeasurement(
        //                 pose_color.estimatedPose.toPose2d(),
        //                 pose_color.timestampSeconds);
        //         colorpub.set(new double[] {
        //             pose_color.estimatedPose.getTranslation().getX(),
        //             pose_color.estimatedPose.getTranslation().getY(),
        //             pose_color.estimatedPose.getRotation().toRotation2d().getDegrees()
        //         });
        //     }
        // }
    }
}
