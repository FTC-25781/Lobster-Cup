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

@Config
@TeleOp(name = "Multi-Blue Object Detector", group = "Concept")
public class CameraOpMode extends LinearOpMode {

    private OpenCvCamera camera;
    private BlueObjectDetectionPipeline pipeline;

    // === Dashboard-adjustable variable ===
    public static double cameraAngleDeg = 31.0;

    @Override
    public void runOpMode() {

        // Set up camera
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

        // Output detection results
        List<Rect> blueObjects = pipeline.getDetectedRects();
        List<Scalar> avgColors = pipeline.getAvgColors();

        if (!blueObjects.isEmpty()) {
            telemetry.addLine(blueObjects.size() + " blue object(s) found:");
            for (int i = 0; i < blueObjects.size(); i++) {
                Rect r = blueObjects.get(i);
                int centerX = r.x + r.width / 2;
                int centerY = r.y + r.height / 2;

                // === Accurate Distance Calculation ===
                double cameraHeightCm = 26.0;
                double verticalFovDeg = 45.0;
                double cameraOffsetCm = 10.0;
                double yOffset = centerY - 240;
                double angleToTarget = cameraAngleDeg + (yOffset / 480.0) * verticalFovDeg;

                // Slant distance using sin (hypotenuse)
                double distanceFromCamera = cameraHeightCm / Math.sin(Math.toRadians(angleToTarget));

                // Ground (horizontal) distance via Pythagorean theorem
                double horizontalDistance = Math.sqrt(
                        Math.pow(distanceFromCamera, 2) - Math.pow(cameraHeightCm, 2));

                // Add camera offset for total robot distance
                double distanceFromRobot = horizontalDistance + cameraOffsetCm;

                // === Color Info (optional) ===
                Scalar avgColor = avgColors.get(i);

                telemetry.addData("Object %d", i + 1);
                telemetry.addData("Center", "X: %d, Y: %d", centerX, centerY);
                telemetry.addData("Size", "Width: %d, Height: %d", r.width, r.height);
                telemetry.addData("Angle to Target", "%.1f deg", angleToTarget);
                telemetry.addData("Distance (Slant)", "%.1f cm", distanceFromCamera);
                telemetry.addData("Distance (Horizontal)", "%.1f cm", horizontalDistance);
                telemetry.addData("Distance from Robot", "%.1f cm", distanceFromRobot);
                telemetry.addData("Avg Color", "B: %.0f G: %.0f R: %.0f",
                        avgColor.val[0], avgColor.val[1], avgColor.val[2]);
            }
        } else {
            telemetry.addLine("No blue objects found.");
        }

        telemetry.update();

        sleep(5000);

        camera.stopStreaming();
        camera.closeCameraDevice();

        while (opModeIsActive()) {
            idle();
        }
    }

    // === OpenCV Pipeline Class ===
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

                    // Average color inside bounding box
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
