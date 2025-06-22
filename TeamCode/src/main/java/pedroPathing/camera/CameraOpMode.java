package pedroPathing.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import pedroPathing.auto.constants.FConstants;
import pedroPathing.auto.constants.LConstants;

@Config
@TeleOp(name = "Camera One Snapshot Drive with Pose Feedback", group = "Concept")
public class CameraOpMode extends LinearOpMode {

    private OpenCvCamera camera;
    private BlueObjectDetectionPipeline pipeline;

    public static double cameraAngleDeg = 31.0;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() {

        // Initialize follower and constants
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        // Setup camera
        int camMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), camMonitorViewId);

        pipeline = new BlueObjectDetectionPipeline();
        camera.setPipeline(pipeline);

        telemetry.addLine("Opening camera...");
        telemetry.update();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        // Wait for first frame processed
        while (opModeIsActive() && !pipeline.hasProcessedFrame()) {
            sleep(50);
        }

        // Get detected objects once
        List<Rect> blueObjects = pipeline.getDetectedRects();

        if (blueObjects.isEmpty()) {
            telemetry.addLine("No blue objects detected at start.");
            telemetry.update();
            follower.setTeleOpMovementVectors(0, 0, 0, true);
            follower.update();
            sleep(3000);
            return;
        }

        // Use first detected object
        Rect r = blueObjects.get(0);
        int centerX = r.x + r.width / 2;
        int centerY = r.y + r.height / 2;

        // Calculate distance and lateral offset
        double cameraHeightCm = 26.0;
        double verticalFovDeg = 45.0;
        double cameraOffsetCm = 10.0;
        double yOffset = centerY - 240;
        double angleToTarget = cameraAngleDeg + (yOffset / 480.0) * verticalFovDeg;

        double distanceFromCamera = cameraHeightCm / Math.sin(Math.toRadians(angleToTarget));
        double horizontalDistance = Math.sqrt(
                Math.pow(distanceFromCamera, 2) - Math.pow(cameraHeightCm, 2));

        double distanceFromRobot = horizontalDistance + cameraOffsetCm;

        // Calculate lateral offset in cm based on pixel offset
        double frameCenterX = 320;
        double pixelErrorX = centerX - frameCenterX;

        double horizontalFovDeg = 60.0; // Approximate for Logitech C270
        double pixelsPerDegree = 640 / horizontalFovDeg;
        double errorXDegrees = pixelErrorX / pixelsPerDegree;

        double lateralOffsetCm = distanceFromRobot * Math.tan(Math.toRadians(errorXDegrees));

        telemetry.addData("Distance to target (cm)", distanceFromRobot);
        telemetry.addData("Lateral offset (cm)", lateralOffsetCm);
        telemetry.update();

        // Target pose relative to start pose
        final double targetX = distanceFromRobot;  // forward
        final double targetY = lateralOffsetCm;    // strafe

        // Tolerance for stopping
        final double toleranceCm = 5.0;

        while (opModeIsActive()) {
            Pose currentPose = follower.getPose();

            // Calculate pose error
            double poseErrorX = targetX - currentPose.getX();
            double poseErrorY = targetY - currentPose.getY();

            // Check if within tolerance, then stop
            if (Math.abs(poseErrorX) < toleranceCm && Math.abs(poseErrorY) < toleranceCm) {
                follower.setTeleOpMovementVectors(0, 0, 0, true);
                telemetry.addLine("Target reached");
                telemetry.update();
                break;
            }

            // Proportional control for smooth driving
            double forwardPower = clamp(poseErrorX * 0.05, -0.5, 0.5);
            double strafePower = clamp(poseErrorY * 0.05, -0.3, 0.3);

            follower.setTeleOpMovementVectors(forwardPower, strafePower, 0, true);

            telemetry.addData("Pose error X (cm)", poseErrorX);
            telemetry.addData("Pose error Y (cm)", poseErrorY);
            telemetry.addData("Forward power", forwardPower);
            telemetry.addData("Strafe power", strafePower);
            telemetry.update();

            follower.update();
            sleep(50);
        }

        // Cleanup
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    // Helper clamp method
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // === OpenCV Pipeline ===
    private static class BlueObjectDetectionPipeline extends OpenCvPipeline {
        private boolean processed = false;
        private final List<Rect> detectedRects = new ArrayList<>();
        private final List<Scalar> avgColors = new ArrayList<>();

        private static final Scalar LOWER_BLUE = new Scalar(100, 150, 50);
        private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);
        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();
        private final Mat rgbaCopy = new Mat();

        public boolean hasProcessedFrame() {
            return processed;
        }

        public List<Rect> getDetectedRects() {
            return detectedRects;
        }

        public List<Scalar> getAvgColors() {
            return avgColors;
        }

        @Override
        public Mat processFrame(Mat input) {
            processed = true;
            input.copyTo(rgbaCopy);

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, LOWER_BLUE, UPPER_BLUE, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            detectedRects.clear();
            avgColors.clear();

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 500) {
                    Rect rect = Imgproc.boundingRect(contour);
                    detectedRects.add(rect);

                    Mat roi = rgbaCopy.submat(rect);
                    Scalar avgColor = Core.mean(roi);
                    avgColors.add(avgColor);
                    roi.release();
                }
            }

            for (Rect rect : detectedRects) {
                Imgproc.rectangle(input, rect, new Scalar(255, 0, 0), 2);
            }

            return input;
        }
    }
}
