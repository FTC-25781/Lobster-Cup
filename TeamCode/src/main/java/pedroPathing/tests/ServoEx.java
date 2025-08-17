package pedroPathing.tests;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ServoEx {
    private Servo servo;
    private HardwareMap hw;

    public ServoEx (HardwareMap hw) {
        this.hw = hw;
    }

    public double getVoltage() {
        double minV = Double.POSITIVE_INFINITY;
        for (VoltageSensor v : hw.getAll(VoltageSensor.class)) {
            double reading = v.getVoltage();
            if (reading > 0) {
                minV = Math.min(minV, reading);
            }
        }
        return (minV == Double.POSITIVE_INFINITY) ? 0.0 : minV;
    }
}
