package pedroPathing.TeleOp.subsystems.intake;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeTurret {

    // === Enum defining all possible turret states ===
    public enum TurretState {
        IDLE,    // No movement or holding position
        LEFT,    // Turret rotated to left position
        CENTER,  // Turret centered
        RIGHT,
        // Turret rotated to right position
    }

    private final Servo turretServo;
    private TurretState currentState;

    // Servo positions corresponding to each state (adjust for your servo range)
    private static final double LEFT_POS = 0.0;
    private static final double CENTER_POS = 0.5;
    private static final double RIGHT_POS = 1.0;

    public IntakeTurret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        currentState = TurretState.IDLE;
        turretServo.setPosition(CENTER_POS); // start centered
    }

    /** Update servo position based on current state */
    public void update() {
        switch (currentState) {
            case LEFT:
                turretServo.setPosition(LEFT_POS);
                break;
            case CENTER:
                turretServo.setPosition(CENTER_POS);
                break;
            case RIGHT:
                turretServo.setPosition(RIGHT_POS);
                break;
            case IDLE:
            default:
                // Do nothing or hold current position
                break;
        }
    }

    /** Set the current turret state and update servo immediately */
    public void setState(TurretState newState) {
        currentState = newState;
        update();
    }

    /** Get the current turret state */
    public TurretState getState() {
        return currentState;
    }

    // Convenience methods to set states quickly
    public void turnLeft() {
        setState(TurretState.LEFT);
    }

    public void turnRight() {
        setState(TurretState.RIGHT);
    }

    public void center() {
        setState(TurretState.CENTER);
    }

    public void idle() {
        setState(TurretState.IDLE);
    }

    /** Return current servo position (0.0 to 1.0) */
    public double getPosition() {
        return turretServo.getPosition();
    }
}

