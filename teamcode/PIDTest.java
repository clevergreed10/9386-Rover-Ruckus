package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDControl.PIDController;

@TeleOp(name = "PIDTest")
@Disabled
public class PIDTest extends OpMode {

    private PIDController controller = new PIDController(0.05, 0.05, 0.1);
    public void init() {

    }

    public void loop() {
        double output = controller.calculate(-5);
        double kP = controller.getkP();
        double kI = controller.getkI();
        double kD = controller.getkD();
        telemetry.addData("output", output);
        telemetry.addData("kP:", kP);
        telemetry.addData("kI:", kI);
        telemetry.addData("kD:", kD);
    }
}
