package pedroPathing.TeleOp.robotControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Turn Point")
public class TurnPoint extends OpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private double pivotWeight = 0.5;  // start centered

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        DcMotor.ZeroPowerBehavior behavior = DcMotor.ZeroPowerBehavior.BRAKE;
        leftFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    @Override
    public void loop() {
        // forward/backward and turn
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        if (gamepad1.x){
            pivotWeight -= 0.01;
        }
        else if(gamepad1.b){
            pivotWeight += 0.01;
        }
        pivotWeight = Math.max(0.0, Math.min(1.0, pivotWeight));

        // adjust motor powers based on pivot weight
        double leftPower = forward + turn * (1 - pivotWeight);
        double rightPower = forward - turn * pivotWeight;

        // apply motor power
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);

        telemetry.addData("Pivot Weight", pivotWeight);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.update();
    }
}
