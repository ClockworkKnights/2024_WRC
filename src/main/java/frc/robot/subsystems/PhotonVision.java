package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class PhotonVision extends SubsystemBase {

    public static boolean vision_enabled = true;

    public static boolean yaw_correction_enabled = false;

    Swerve m_swerve = TunerConstants.DriveTrain;
    Pigeon2 m_gyro = m_swerve.getPigeon2();

    public static PhotonCamera cam_gray = new PhotonCamera("gray");
    public static PhotonCamera cam_color = new PhotonCamera("color");

    public static double note_detection_yaw_error = 0;

    private final DoubleArrayPublisher photonvisionpub = NetworkTableInstance.getDefault()
            .getTable("Pose").getDoubleArrayTopic("photonvision")
            .publish();

    private final BooleanPublisher gray_alive_pub = NetworkTableInstance.getDefault()
            .getTable("Diagnostics").getSubTable("PhotonVision").getBooleanTopic("Gray Cam Alive")
            .publish();
    private final BooleanPublisher color_alive_pub = NetworkTableInstance.getDefault()
            .getTable("Diagnostics").getSubTable("PhotonVision").getBooleanTopic("Color Cam Alive")
            .publish();

    private final NetworkTable tableNote = NetworkTableInstance.getDefault().getTable("Debug").getSubTable("Aimer").getSubTable("NoteDetection");
    private final DoublePublisher noteDetectionYawErrorPub = tableNote.getDoubleTopic("Yaw Error").publish();

    public void update_apriltag() {
        boolean doRejectUpdate = !vision_enabled;
        var result = cam_gray.getLatestResult();
        var timestamp = result.getTimestampSeconds();
        if (Timer.getFPGATimestamp() - timestamp > 1) {
            gray_alive_pub.set(false);
            return;
        } else {
            gray_alive_pub.set(true);
        }
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

            if (yaw_correction_enabled) {
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

    public void update_note_detection() {
        var result = cam_color.getLatestResult();
        var results = result.getTargets();
        var timestamp = result.getTimestampSeconds();
        if (Timer.getFPGATimestamp() - timestamp > 1) {
            color_alive_pub.set(false);
            return;
        } else {
            color_alive_pub.set(true);
        }
        if (results.size() > 0) {
            int best_target_idx = -1;
            double best_target_x = 0;
            double best_target_y = 0;
            int count = 0;
            for (var target : results) {
                var corners = target.getMinAreaRectCorners();
                double x = (corners.get(0).x + corners.get(1).x + corners.get(2).x + corners.get(3).x) / 4;
                double y = (corners.get(0).y + corners.get(1).y + corners.get(2).y + corners.get(3).y) / 4;
                // var width = Math.max(Math.abs(corners.get(0).x - corners.get(1).x),
                //         Math.abs(corners.get(0).x - corners.get(2).x));
                if (best_target_idx == -1) {
                    best_target_x = x;
                    best_target_y = y;
                }
                if (y > best_target_y) {
                    best_target_idx = count;
                    best_target_x = x;
                    best_target_y = y;
                }
                // var height = Math.max(Math.abs(corners.get(0).y - corners.get(1).y),
                // Math.abs(corners.get(0).y - corners.get(2).y));
                count += 1;
            }
            // System.out.println("Best Target: " + best_target_x + ", " + best_target_y);
            note_detection_yaw_error = getNoteDetectionYawError(best_target_x, best_target_y);
            noteDetectionYawErrorPub.set(note_detection_yaw_error);
        } else {
            note_detection_yaw_error = 0;
            noteDetectionYawErrorPub.set(note_detection_yaw_error);
        }
    }

    @Override
    public void periodic() {
        update_apriltag();
        update_note_detection();
    }

    public double getNoteDetectionYawError(double x, double y) {
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

        // return target-now
        double raw = (-1.119 * x + 2026.4) - y;
        return raw / 1000 * 45;
    }

}
