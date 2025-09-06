package pedroPathing.TeleOp.robotControl;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import java.util.ArrayList;


@TeleOp(name="Sensor Fusion OpenCV and IMU")
public class SensorFusion extends LinearOpMode {

    private IMU imu;
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private OpenCvCamera webcam;
    private ObstacleDetectionPipeline pipeline;
    private DistanceSensor distanceSensor;

    private final double thresholdDistance = 30.0;
    private final double correctionGain = 0.015;
    private final double turnGain = 0.01;

    double targetHeading;


    public void runOpMode(){

        imu = hardwareMap.get(IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))
        );

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);


        pipeline = new ObstacleDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });


        waitForStart();
        targetHeading = getHeading();

        while (opModeIsActive()) {
            double currentDistance = distanceSensor.getDistance(DistanceUnit.CM);
            Rect obstacle = pipeline.getLargestObstacle();

            telemetry.addData("Distance (cm)", "%.2f", currentDistance);
            telemetry.addData("IMU Heading", "%.2f", getHeading());

            if (obstacle != null && obstacle.width > 50 && currentDistance < thresholdDistance) {
                stopMotors();
                telemetry.addLine("Obstacle Detected & Close");
                telemetry.addData("Obstacle X", obstacle.x);
                telemetry.addData("Obstacle Width", obstacle.width);

                if (obstacle.x + obstacle.width / 2 < 320) { // Obstacle on left side
                    turnToHeading(targetHeading + 30); // Turn right
                } else { // Obstacle on right side
                    turnToHeading(targetHeading - 30); // Turn left
                }

                driveForward(0.1, 500);
                turnToHeading(targetHeading);
            } else {
                driveWithIMU(0.1, targetHeading);
                telemetry.addLine("Driving Forward");
            }

            telemetry.update();
        }
    }



    double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    void driveWithIMU(double power, double targetHeading) {
        double headingError = targetHeading - getHeading();
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;

        double correction = headingError * correctionGain;

        double leftPower = power - correction;
        double rightPower = power + correction;

        leftPower = Math.max(-1, Math.min(leftPower, 1));
        rightPower = Math.max(-1, Math.min(rightPower, 1));

        setMotorPowers(leftPower, rightPower);
    }

    void driveForward(double power, long time)  {
        setMotorPowers(power, power);
        sleep(time);
        stopMotors();
    }

    void turnToHeading(double target) {
        while (opModeIsActive()) {
            double error = target - getHeading();

            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            if (Math.abs(error) <= 2) {
                break;
            }


            double turnPower = error * turnGain;

            turnPower = Math.max(-0.3, Math.min(turnPower, 0.3));

            setMotorPowers(turnPower, -turnPower);
        }
        stopMotors();
    }

    void setMotorPowers(double left, double right) {
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);
    }

    void stopMotors() {
        setMotorPowers(0, 0);
    }

    public static class ObstacleDetectionPipeline extends OpenCvPipeline {
        private Mat hsv = new Mat();
        private Mat mask = new Mat();
        private Mat hierarchy = new Mat();
        private ArrayList<MatOfPoint> contours = new ArrayList<>();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lower = new Scalar(20, 100, 100);
            Scalar upper = new Scalar(40, 255, 255);

            Core.inRange(hsv, lower, upper, mask);

            contours.clear();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                if (rect.area() > 500) {
                    Imgproc.rectangle(input, rect, new Scalar(0, 255, 0), 2);
                }
            }
            return mask;
        }

        public Rect getLargestObstacle() {
            double maxArea = 0;
            Rect largest = null;

            ArrayList<MatOfPoint> contoursCopy;
            synchronized(this) {
                contoursCopy = new ArrayList<>(contours);
            }

            for (MatOfPoint contour : contoursCopy) {
                Rect rect = Imgproc.boundingRect(contour);
                if (rect.area() > maxArea) {
                    maxArea = rect.area();
                    largest = rect;
                }
            }
            return largest;
        }
    }
}









