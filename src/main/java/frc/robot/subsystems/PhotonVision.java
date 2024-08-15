package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class PhotonVision extends SubsystemBase {

    public static boolean vision_yaw = false;

    Swerve m_swerve = TunerConstants.DriveTrain;
    Pigeon2 m_gyro = new Pigeon2(20, "canivore");

    public static PhotonCamera cam_gray = new PhotonCamera("gray");
    public static PhotonCamera cam_color = new PhotonCamera("color");

    public static double yaw_error = 0;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher photonvisionpub = table.getDoubleArrayTopic("photonvision").publish();

    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    // public static final AprilTagFieldLayout aprilTagFieldLayout =
    // AprilTagFields.k2024Crescendo
    // .loadAprilTagLayoutField();
    // public static final Transform3d robot_to_cam_gray = new Transform3d(new
    // Translation3d(0.33, 0.23, 0.32),
    // new Rotation3d(0, Math.PI / 6., Math.PI/6*7));

    // public static PhotonPoseEstimator photonPoseEstimator = new
    // PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam_gray, robot_to_cam_gray);

    @Override
    public void periodic() {
        {
            boolean doRejectUpdate = false;
            var result = cam_gray.getLatestResult();
            var timestamp = result.getTimestampSeconds();
            var count_tags = result.getTargets().size();
            if (count_tags == 1) {
                if (result.getBestTarget().getPoseAmbiguity() > .7) {
                    doRejectUpdate = true;
                }
                var translation = result.getBestTarget().getBestCameraToTarget();
                var norm = translation.getX() * translation.getX() + translation.getY() * translation.getY()
                        + translation.getZ() * translation.getZ();
                if (norm > 3) {
                    doRejectUpdate = true;
                }
            }
            if (count_tags == 0) {
                doRejectUpdate = true;
            }
            if (result.getMultiTagResult().estimatedPose.isPresent && !doRejectUpdate) {
                Transform3d field_to_cam_pose = result.getMultiTagResult().estimatedPose.best;
                photonvisionpub.set(new double[] {
                        field_to_cam_pose.getX(),
                        field_to_cam_pose.getY(),
                        new Rotation2d(-field_to_cam_pose.getRotation().getAngle()).getDegrees()
                });
                var x = field_to_cam_pose.getX();
                var y = field_to_cam_pose.getY();
                var yaw = m_swerve.getPose().getRotation().getRadians();
                var dx = 0.23;
                var dy = -0.33;
                double x_new = x + dx * Math.cos(yaw) + dy * Math.sin(yaw);
                double y_new = y + dx * Math.sin(yaw) + dy * Math.cos(yaw);

                if (vision_yaw) {
                    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 6));
                    m_swerve.addVisionMeasurement(
                            new Pose2d(x_new, y_new,
                                    new Rotation2d(field_to_cam_pose.getRotation().getAngle())
                                            .minus(new Rotation2d(Math.PI / 6 * 5))),
                            timestamp);
                } else {
                    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 99999999.));
                    m_swerve.addVisionMeasurement(
                            new Pose2d(x_new, y_new,
                                    m_swerve.getPigeon2().getRotation2d()),
                            // new Rotation2d(field_to_cam_pose.getRotation().getAngle()).minus(new
                            // Rotation2d(Math.PI/6*5))),
                            timestamp);
                }
            }
        }

        {
            var result = cam_color.getLatestResult();
            var results = result.getTargets();
            var timestamp = result.getTimestampSeconds();
            if (results.size() > 0) {
                int best_target_idx = -1;
                double best_target_x = 0;
                double best_target_y = 0;
                int count = 0;
                for (var target : results) {
                    var corners = target.getMinAreaRectCorners();
                    double x = (corners.get(0).x + corners.get(1).x + corners.get(2).x + corners.get(3).x) / 4;
                    double y = (corners.get(0).y + corners.get(1).y + corners.get(2).y + corners.get(3).y) / 4;
                    var width = Math.max(Math.abs(corners.get(0).x - corners.get(1).x),
                            Math.abs(corners.get(0).x - corners.get(2).x));
                    if (best_target_idx == -1) {
                        best_target_x = x;
                        best_target_y = y;
                    }
                    if (y < best_target_y) {
                        best_target_idx = count;
                        best_target_x = x;
                        best_target_y = y;
                    }
                    // var height = Math.max(Math.abs(corners.get(0).y - corners.get(1).y),
                    // Math.abs(corners.get(0).y - corners.get(2).y));
                    count += 1;
                }
                // System.out.println("Best Target: " + best_target_x + ", " + best_target_y);
                yaw_error = getYawError(best_target_x, best_target_y);
            } else {
                yaw_error = 0;
            }
        }
    }

    // Crosshair keypoints
    // x y
    // 820 1062
    // 1075 885
    // 1160 766
    // 1224 711
    // 1252 688

    // y = -1.119x + 2026.4
    // if y > -1.119x + 2026.4, then the target is to the left of the crosshair,
    // yaw_error > 0
    // if y < -1.119x + 2026.4, then the target is to the right of the crosshair,
    // yaw_error < 0

    public double getYawError(double x, double y) {
        // return target-now
        return y - (-1.119 * x + 2026.4);
    }

    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d
    // prevEstimatedRobotPose) {
    // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    // return photonPoseEstimator.update();
    // }
}
