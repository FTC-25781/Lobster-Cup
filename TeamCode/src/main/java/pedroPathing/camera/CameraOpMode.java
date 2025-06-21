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

// Imports for Follower and Pose
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import pedroPathing.auto.constants.FConstants;
import pedroPathing.auto.constants.LConstants;

@Config
@TeleOp(name = "Camera Drive Forward + Strafe", group = "Concept")
public class CameraOpMode extends LinearOpMode {

    private OpenCvCamera camera;
    private BlueObjectDetectionPipeline pipeline;

    public static double cameraAngleDeg = 31.0;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() {

        // Follower setup
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        // Camera setup
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

        // Wait until first frame is processed
        while (opModeIsActive() && !pipeline.hasProcessedFrame()) {
            sleep(50);
        }

        // Control tuning constants
        final double strafeGain = 0.003;    // How aggressively to strafe (lower = smoother)
        final double driveGain = 0.05;    // Forward speed scale
        final double maxStrafePower = 0.3; // Max strafe power
        final double maxForwardPower = 0.5; // Max forward power
        final double minForwardPower = 0.1; // Minimum forward to overcome friction
        final double stopDistanceCm = 10; // How close to stop
        final double deadband = 15;       // pixels deadband for strafe

        // Main loop — camera streaming stays active!
        while (opModeIsActive()) {

            List<Rect> blueObjects = pipeline.getDetectedRects();

            if (!blueObjects.isEmpty()) {
                Rect r = blueObjects.get(0);
                int centerX = r.x + r.width / 2;
                int centerY = r.y + r.height / 2;

                // Distance calculation
                double cameraHeightCm = 26.0;
                double verticalFovDeg = 45.0;
                double cameraOffsetCm = 10.0;
                double yOffset = centerY - 240;
                double angleToTarget = cameraAngleDeg + (yOffset / 480.0) * verticalFovDeg;

                double distanceFromCamera = cameraHeightCm / Math.sin(Math.toRadians(angleToTarget));
                double horizontalDistance = Math.sqrt(
                        Math.pow(distanceFromCamera, 2) - Math.pow(cameraHeightCm, 2));

                double distanceFromRobot = horizontalDistance + cameraOffsetCm;

                // Strafe control (based on horizontal error)
                double frameCenterX = 320;
                double errorX = centerX - frameCenterX;

                double strafe = 0;
                if (Math.abs(errorX) > deadband) {
                    strafe = errorX * strafeGain;
                    // Clamp strafe power
                    strafe = Math.max(-maxStrafePower, Math.min(strafe, maxStrafePower));
                }

                // Forward control
                double forward = 0;
                if (distanceFromRobot > stopDistanceCm) {
                    forward = distanceFromRobot * driveGain;
                    // Clamp forward power between min and max
                    forward = Math.min(maxForwardPower, Math.max(minForwardPower, forward));
                }

                if (distanceFromRobot <= stopDistanceCm) {
                    forward = 0;
                    strafe = 0;
                    telemetry.addLine("Object reached.");
                } else {
                    telemetry.addLine("Driving toward object...");
                }

                double turn = 0;  // no turning at all!

                follower.setTeleOpMovementVectors(forward, strafe, turn, true);

                // Telemetry for debugging
                telemetry.addData("errorX (pixels)", errorX);
                telemetry.addData("strafe command", strafe);
                telemetry.addData("forward command", forward);
                telemetry.addData("Distance (cm)", distanceFromRobot);
            } else {
                follower.setTeleOpMovementVectors(0, 0, 0, true);
                telemetry.addLine("No blue objects detected.");
            }

            follower.update();
            telemetry.update();
            sleep(50);
        }

        // Stop camera cleanly after opmode ends
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    // === Pipeline ===
    private static class BlueObjectDetectionPipeline extends OpenCvPipeline {
        private boolean processed = false;
        private List<Rect> detectedRects = new ArrayList<>();
        private List<Scalar> avgColors = new ArrayList<>();

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
