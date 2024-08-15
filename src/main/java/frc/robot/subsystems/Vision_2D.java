package frc.robot.subsystems;

import java.util.Vector;

import org.ejml.data.Matrix;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Vision_2D extends SubsystemBase {

    public class EKF {
        public Matrix QEst;
        public Matrix PYEst;
        public Pair<Integer, Pose2d> Map;

        public static double angle_wrap(double a) {
            if (a > Math.PI) {
                a = a - 2 * Math.PI;
            } else if (a < -Math.PI) {
                a = a + 2 * Math.PI;
            }
            return a;
        }

        public static SimpleMatrix tcomp(SimpleMatrix tab, SimpleMatrix tbc) {
            // Ensure the matrices are 2D

            // Compute the angle sum
            double result = tab.get(2, 0) + tbc.get(2, 0);
            result = angle_wrap(result);

            // Compute sine and cosine
            double s = Math.sin(tab.get(2, 0));
            double c = Math.cos(tab.get(2, 0));

            // Create the rotation matrix
            double[][] rotationArray = {
                    { c, -s },
                    { s, c }
            };
            SimpleMatrix rotationMatrix = new SimpleMatrix(rotationArray);

            // Perform the matrix multiplication and addition
            SimpleMatrix tac = tab.rows(0, 2).plus(rotationMatrix.mult(tbc.rows(0, 2)));

            // Append the result as the third row
            tac = tac.concatRows(new SimpleMatrix(1, 2, true, new double[] { result, 0 }));

            return tac;
        }

        public static SimpleMatrix getOdomPose() {
            var pose = TunerConstants.DriveTrain.getPose();
            return new SimpleMatrix(3, 1, true,
                    new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
        }

        public static SimpleMatrix getOdomStep(double time_step) {
            var vel = TunerConstants.DriveTrain.getSpeeds();
            return new SimpleMatrix(3, 1, true, new double[] { vel.vxMetersPerSecond * time_step,
                    vel.vyMetersPerSecond * time_step, vel.omegaRadiansPerSecond * time_step });
        }

        public class Observation {
            public int id;
            public double dist;
            public double angle_from_heading;
        }

        public static Vector<Observation> getObservations() {
            var observations = new Vector<Observation>();
            double limelight_latency = LimelightHelpers.getLatency_Capture("limelight")
                    + LimelightHelpers.getLatency_Pipeline("limelight");
            var limelight_results = LimelightHelpers.getRawDetections("limelight");
            // TODO
            return observations;
        }
    }

    public Vision_2D() {
    }

    @Override
    public void periodic() {

    }

}
