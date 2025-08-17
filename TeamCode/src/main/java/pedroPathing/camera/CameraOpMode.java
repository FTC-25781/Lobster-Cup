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
    // cool camera
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
        final int maxLostFrames = 100;
        final double stopBeforeCm = 15.0;
        final double toleranceCm = 5.0;

        while (opModeIsActive()) {

            if (!pipeline.hasProcessedFrame() && gamepad1.dpad_down) {
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

            Rect r = pipeline.getFocusedRect();
            if (r == null) continue;

            int centerX = r.x + r.width / 2;
            int centerY = r.y + r.height / 2;

            // === Robot Front Center (Green Dot) Image Coordinates ===
            double in2px = 10.0; // 10 pixels = 1 inch
            double px2cm = 2.54 / in2px;

            double cameraVertOffsetIn = 9.0;     // robot front is 9 inches ahead of camera
            double cameraHorizOffsetIn = -7.0;   // robot front is 5 inches left of camera

            double robotFrontCenterX = 320 + cameraHorizOffsetIn * in2px;
            double robotFrontCenterY = 240 + cameraVertOffsetIn * in2px;

            // === Compute pixel difference from robot center to object center ===
            double dxPixels = centerX - robotFrontCenterX;
            double dyPixels = centerY - robotFrontCenterY;

            // === Convert pixel difference to real-world cm ===
            double dxCm = dxPixels * px2cm;
            double dyCm = dyPixels * px2cm;

            Pose currentPose = follower.getPose();

            double targetX = currentPose.getX() + dyCm;  // Forward direction
            double targetY = currentPose.getY() + dxCm;  // Strafe direction

            double poseErrorX = targetX - currentPose.getX();  // Forward error
            double poseErrorY = targetY - currentPose.getY();  // Strafe error

            if (Math.abs(poseErrorX) < toleranceCm && Math.abs(poseErrorY) < toleranceCm) {
                follower.setTeleOpMovementVectors(0, 0, 0, true);
                telemetry.addLine("Aligned with object");
                telemetry.update();
                break;
            }

            double forwardPower = clamp(poseErrorX * 0.05, -0.5, 0.5);
            double strafePower = clamp(poseErrorY * 0.05, -0.3, 0.3);

            follower.setTeleOpMovementVectors(-forwardPower * 0.55, -strafePower * 0.55, 0, true);
            follower.update();

            telemetry.addData("dx (cm)", dxCm);
            telemetry.addData("dy (cm)", dyCm);
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

        private Rect focusedRect = null;

        public Rect getFocusedRect() {
            return focusedRect;
        }

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

            double cameraVertOffsetIn = -7.0;
            double cameraHorizontalOffsetIn = 9.0;

            double in2px = 10.0;

            double offsetX = cameraVertOffsetIn * in2px;
            double offsetY = cameraHorizontalOffsetIn * in2px;

            int cameraCenterX = 320;
            int cameraCenterY = 240;

            double robotFrontCenterX = cameraCenterX + offsetX;
            double robotFrontCenterY = cameraCenterY + offsetY;

            Imgproc.circle(input, new Point(robotFrontCenterX, robotFrontCenterY), 8, new Scalar(0, 255, 0), -1);

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

            focusedRect = null;
            double minDistance = Double.MAX_VALUE;

            double robotIRLFrontCenterX = robotFrontCenterX * in2px;
            double robotIRLFrontCenterY = robotFrontCenterY * in2px;

            for (Rect rect : detectedRects) {
                int centerX = rect.x + rect.width / 2;
                int centerY = rect.y + rect.height / 2;

                double dx = centerX - robotIRLFrontCenterX;
                double dy = centerY - robotIRLFrontCenterY;

                double distance = Math.sqrt(dx * dx + dy * dy);

                if (distance < minDistance) {
                    minDistance = distance;
                    focusedRect = rect;
                }

                if (focusedRect != null) {
                    Imgproc.rectangle(input, focusedRect, new Scalar(255, 0, 0), 2);
                }
            }

            return input;
        }
    }
}
