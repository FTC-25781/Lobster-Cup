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
@TeleOp(name = "Camera Object Drive (Safe Distance)", group = "Concept")
public class CameraOpMode extends LinearOpMode {

    private OpenCvCamera camera;
    private BlueObjectDetectionPipeline pipeline;

    public static final double CAMERA_TILT = 31.0;
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

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

        int framesWithoutDetection = 0;
        final int maxLostFrames = 10;
        final double stopBeforeCm = 15.0;
        final double toleranceCm = 5.0;

        while (opModeIsActive()) {

            if (!pipeline.hasProcessedFrame()) {
                sleep(50);
                continue;
            }

            List<Rect> blueObjects = pipeline.getDetectedRects();
            if (blueObjects.isEmpty()) {
                framesWithoutDetection++;
                telemetry.addData("Frames without detection", framesWithoutDetection);
                telemetry.update();

                if (framesWithoutDetection > maxLostFrames) {
                    follower.setTeleOpMovementVectors(0, 0, 0, true);
                    telemetry.addLine("Lost object for too long. Stopping.");
                    telemetry.update();
                    break;
                }
                continue;
            } else {
                framesWithoutDetection = 0;
            }

            Rect r = blueObjects.get(0);
            int centerX = r.x + r.width / 2;
            int centerY = r.y + r.height / 2;

            // Constants
            // Camera height from the base
            double cameraHeightCm = 26.0;
            // Logitec FOV
            double verticalFovDeg = 45.0;
            double horizontalFovDeg = 60.0;
            //
            double cameraOffsetCm = 15.24;

            // Camera
            // CenterY is the center of the object
            // 240 is half the screen height
            double yOffset = centerY - 240;

            // Angle to target from CAM frame of reference on z-axis
            double angleToTarget = CAMERA_TILT + (yOffset / 480.0) * verticalFovDeg;

            // X distance from the object to CAM
            double distanceFromCamera = cameraHeightCm / Math.sin(Math.toRadians(angleToTarget));

            double horizontalDistance = Math.sqrt(
                    Math.pow(distanceFromCamera, 2) - Math.pow(cameraHeightCm, 2));
            double distanceFromRobot = horizontalDistance + cameraOffsetCm - stopBeforeCm;

            double pixelErrorX = centerX - 320;
            double pixelsPerDegree = 640 / horizontalFovDeg;
            double errorXDegrees = pixelErrorX / pixelsPerDegree;
            double lateralOffsetCm = distanceFromRobot * Math.tan(Math.toRadians(errorXDegrees)) + 15.24;

            Pose currentPose = follower.getPose();
            double poseErrorX = distanceFromRobot - currentPose.getX();
            double poseErrorY = lateralOffsetCm - currentPose.getY();

            if (Math.abs(poseErrorX) < toleranceCm && Math.abs(poseErrorY) < toleranceCm) {
                follower.setTeleOpMovementVectors(0, 0, 0, true);
                telemetry.addLine("Target reached (safe distance)");
                telemetry.update();
                break;
            }

            double forwardPower = clamp(poseErrorX * 0.05, -0.5, 0.5);
            double strafePower = clamp(poseErrorY * 0.05, -0.3, 0.3);

            follower.setTeleOpMovementVectors(forwardPower, strafePower, 0, true);
            follower.update();

            telemetry.addData("Distance to target (cm)", distanceFromRobot);
            telemetry.addData("Lateral offset (cm)", lateralOffsetCm);
            telemetry.addData("Pose error X", poseErrorX);
            telemetry.addData("Pose error Y", poseErrorY);
            telemetry.addData("Forward power", forwardPower);
            telemetry.addData("Strafe power", strafePower);
            telemetry.update();

            sleep(50);
        }

        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // === Blue Object Detection Pipeline ===
    private static class BlueObjectDetectionPipeline extends OpenCvPipeline {
        private boolean processed = false;
        private final List<Rect> detectedRects = new ArrayList<>();
        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();
        private final Mat rgbaCopy = new Mat();

        private static final Scalar LOWER_BLUE = new Scalar(100, 150, 50);
        private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);

        public boolean hasProcessedFrame() {
            return processed;
        }

        public List<Rect> getDetectedRects() {
            return detectedRects;
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

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 500) {
                    Rect rect = Imgproc.boundingRect(contour);
                    detectedRects.add(rect);
                }
            }

            for (Rect rect : detectedRects) {
                Imgproc.rectangle(input, rect, new Scalar(255, 0, 0), 2);
            }

            return input;
        }
    }
}
