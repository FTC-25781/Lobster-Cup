package pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test", group = "Linear OpMode")
public class MotorTest extends LinearOpMode {
    private DcMotor motor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor = hardwareMap.get(DcMotor.class, "intakeSlides");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                motor.setTargetPosition(-355);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.5);

                while(motor.isBusy()) {
                    telemetry.addData("Power: ", motor.getPower());
                    telemetry.addData("Encoder Position", motor.getCurrentPosition());
                    telemetry.update();
                }

                motor.setPower(0);
            }

            if (gamepad1.dpad_down) {
                motor.setTargetPosition(-76);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(-0.5);

                while(motor.isBusy()) {
                    telemetry.addData("Power: ", motor.getPower());
                    telemetry.addData("Encoder Position", motor.getCurrentPosition());
                    telemetry.update();
                }

                motor.setPower(0);
            }
        }
    }
}
