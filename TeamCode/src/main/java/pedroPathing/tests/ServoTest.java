package pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Linear OpMode")
public class ServoTest extends LinearOpMode {
    private Servo servo1;
    private Servo servo2;

    private static final double SERVO_INCREMENT = 0.01;
    private boolean going_down = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition((servo1.getPosition() + gamepad1.left_stick_y) * 0.001);
            servo2.setPosition((servo2.getPosition() + gamepad1.left_stick_y) * 0.001);
        }
    }
}
