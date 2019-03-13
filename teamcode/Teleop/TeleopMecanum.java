package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.EEBotHardware;
import org.firstinspires.ftc.teamcode.PIDControl.PIDController;
import org.firstinspires.ftc.teamcode.Util.Gamepad.Button;

/**
 *  9386 Mecanum Drive Program
 *
 *
 *   - - - CONTROLS - - -
 *
 *   gamepad1:
 *      left stick: moves robot
 *      right stick: rotates robot
 *
 *      right trigger: controls intake
 *      left trigger: controls intake
 *
 *   gamepad2:
 *      TODO: UPDATE
 *
 */
@TeleOp(name="Teleop: Mec", group="Robot")

public class TeleopMecanum extends OpMode {

    private EEBotHardware robot = new EEBotHardware();

    double kR = 0.95;

    private double forward, strafe, rotate;
    private double front_right, front_left, back_right, back_left;
    private double armPower, liftPower  = 0;
    private double winchPower, intakePower = 0;
    private double WHEEL_DAMP = 1.0;
    private double desiredArmPos = 0;

    private int SPOOL_SCORE_POS;
    private int SPOOL_START_POS;

    private Button pad2A = new Button();
    private Button pad2B = new Button();
    private Button pad2X = new Button();
    private Button pad2Y = new Button();

    private Button pad1Y = new Button();


    private Button leftBump2 = new Button();
    private Button rightBump2 = new Button();

    private PIDController armController = new PIDController(1/50, 1/100, 1/400);

    @Override // @Override tells the computer we intend to override OpMode's method init()
    public void init() {
        robot.init(hardwareMap);

        //robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //armController.setMaxIntegral(100); // CHECK THIS

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        //Update Button Data
        pad2A.update(gamepad2.a);
        pad2B.update(gamepad2.b);
        pad2X.update(gamepad2.x);
        pad2Y.update(gamepad2.y);
        leftBump2.update(gamepad2.left_bumper);
        rightBump2.update(gamepad2.right_bumper);

        // Robot Movement Logic
        forward = -gamepad1.left_stick_y;

        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
            strafe = -gamepad1.left_trigger + gamepad1.right_trigger;
        } else {
            strafe = gamepad1.left_stick_x;
        }

        pad1Y.update(gamepad1.y);

        if (pad1Y.isToggle()) {
            if (WHEEL_DAMP == 0.6) {
                WHEEL_DAMP = 1.0;
            } else {
                WHEEL_DAMP = 0.6;
            }
        }

        rotate  = kR * gamepad1.right_stick_x;

        front_left  = forward + rotate + strafe;
        front_right = forward - rotate - strafe;
        back_left   = forward + rotate - strafe;
        back_right  = forward - rotate + strafe;

        double max = Math.abs(front_left);
        if (Math.abs(front_right) > max) max = Math.abs(front_right);
        if (Math.abs(back_left) > max) max = Math.abs(back_left);
        if (Math.abs(back_right) > max) max = Math.abs(back_right);

        if (max > 1) {
            front_left  /= max;
            front_right /= max;
            back_left   /= max;
            back_right  /= max;
        }

        robot.wheel1.setPower(front_right * WHEEL_DAMP);
        robot.wheel2.setPower(front_left * WHEEL_DAMP);
        robot.wheel3.setPower(back_left * WHEEL_DAMP);
        robot.wheel4.setPower(back_right * WHEEL_DAMP);

        // Scoring logic
        armPower = gamepad2.right_stick_y;

        //Set spool to scoring position
        if (gamepad2.a){
            robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.winch.setTargetPosition(SPOOL_SCORE_POS);
            /*
             if (robot.winch.getCurrentPosition() < robot.winch.getTargetPosition()){
                robot.winch.setPower(1);
            }
            else if (robot.winch.getCurrentPosition() > -1346){
                robot.winch.setPower(-1);
            }
            else{
                robot.winch.setPower(0);
            }
             */
        } else if (gamepad2.b){
            robot.winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.winch.setPower(0);
        } else if (gamepad2.x){
            robot.winch.setPower(0);
            robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.winch.setPower(0);
            robot.winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad2.dpad_right) {
            // spool scoring position
            SPOOL_SCORE_POS = robot.winch.getCurrentPosition();
        } else if (gamepad2.dpad_left) {
            SPOOL_START_POS = robot.winch.getCurrentPosition();
        }

        // find spool power
        if (gamepad2.right_trigger > 0.1) {
            winchPower = -gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0.1) {
            winchPower = gamepad2.left_trigger;
        } else {
            winchPower = 0;
        }


        if (gamepad1.x) {
            intakePower = -1;
        }
        if (gamepad1.a) {
            intakePower = 0;
        }
        if (gamepad1.b){
            intakePower = 1;
        }

        // THIS OVERRIDES OLD BUTTON USAGE
        if (gamepad1.right_bumper) {
            intakePower = 1;
        }
        if (gamepad1.left_bumper) {
            intakePower = -1;
        }

        // Moves the Team Marker gear.
        if (gamepad2.y) {
            robot.pusher.setPosition(0);
        }
        if (gamepad2.x){
            robot.pusher.setPosition(1);
        }

        if (gamepad1.dpad_left){
            robot.stopper.setPosition(1);
        }
        if (gamepad1.dpad_right){
            robot.stopper.setPosition(0);
        }

        //Robot Lift
        liftPower = gamepad2.left_stick_y * -1;
        telemetry.addData("Arm position: ", robot.armMotor.getCurrentPosition());

        // Winch extension

        robot.armMotor.setPower(armPower);
        robot.liftMotor.setPower(liftPower);

        robot.intakeMotor.setPower(intakePower);
        robot.winch.setPower(winchPower);
        robot.liftMotor.setPower(liftPower);


        telemetry.addData("Arm Value: ", armPower);
        telemetry.addData("Lift Value: ", liftPower);
        telemetry.addData("Intake Value: ", intakePower);
        telemetry.addData("Winch Value: ", winchPower);
        //telemetry.addData("Winch Position: ", robot.winch.getCurrentPosition());
        //telemetry.addData("front_right Value: ", front_right);
        //telemetry.addData("front_left Value: ", front_left);
        //telemetry.addData("back_right Value: ", back_right);
        //telemetry.addData("back_left Value: ", back_left);
        telemetry.update();
    }

    private double getMotorPower(double stick) {
        // experimenting with sinusoidal curve
        double finalPower = -1 * Math.sin(stick*Math.PI/2); // sin(pi*x/2) >>> period 2, [-1,1]
        return finalPower;
    }
}
