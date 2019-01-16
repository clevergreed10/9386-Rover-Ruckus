package org.firstinspires.ftc.teamcode.AutoPrograms;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.EEBotHardware;
import org.firstinspires.ftc.teamcode.PIDControl.PController;

@Autonomous(name="Auto: Full-Crater2")

public class AutoFullCrater2 extends LinearOpMode {
    EEBotHardware bot = new EEBotHardware();

    final double WHEEL_CIRCUMFERENCE = 4*Math.PI;
    final double TICKS_PER_ROTATION  = EEBotHardware.NEVEREST20_TPR;
    final double TICKS_PER_INCH      = TICKS_PER_ROTATION/WHEEL_CIRCUMFERENCE;
    // Rev HD Hex Motor has 56 ticks per ENCODER revolution, geared at 1:40 creates 2,240 ticks per SHAFT revolution

    private double DRIVE_kP = 1/20;
    private final double DRIVE_kI = 1/40;
    private final double DRIVE_kD = 1/15;

    private DcMotor wheel1;
    private DcMotor wheel2;
    //private DcMotor wheel3;
    //private DcMotor wheel4;

    private GoldAlignDetector detector = new GoldAlignDetector();
    private int goldSide = 1; //-1 = NONE, 0 = LEFT, 1 = CENTER, 2 = RIGHT

    @Override
    public void runOpMode() {
        //initialize

        bot.init(hardwareMap);
        //detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, false);
//        bot.liftLockServo.setPosition(bot.LIFT_LOCK_CLOSE);
        wheel1 = bot.wheel1;
        wheel2 = bot.wheel2;
        //wheel3 = bot.wheel3;
        //wheel4 = bot.wheel4;

        waitForStart();

        detector.enable();
        //run commands
        telemetry.addLine("OpMode is Active");
        telemetry.update();

        // lower from lander
        ////Release claw lock
//        bot.liftLockServo.setPosition(bot.LIFT_LOCK_OPEN);
//
//        ////Extend lander claw
//        gyroHold(4);
//        bot.liftMotor.setPower(0.5);
//        gyroHold(3);
//        bot.liftMotor.setPower(0.0);
//
//        ////Turn 15* clockwise (right)
//        bot.wheel2.setPower(-0.5);
//        gyroHold(1);
//        bot.wheel2.setPower(0.0);
//
//        ////Retract lander lift
//        bot.liftMotor.setPower(-0.5);
//        gyroHold(3);
//        bot.liftMotor.setPower(0.0);


        // read minerals
        gyroDrive(0, -0.5, 6, 2.0);
        gyroHold(5);

        if (detector.isFound()) {
            double goldPos = detector.getXPosition();

            if (goldPos < 200) { // 0 is left, 2 is right, 1 is center
                goldSide = 0;
            } else if (goldPos > 450) {
                goldSide = 2;
            } else {
                goldSide = 1;
            }
            telemetry.addData("Side", goldSide);
            telemetry.addData("Pos", goldPos);
            telemetry.update();
            gyroHold(5);
        }

        // push off correct mineral, back up
        if (goldSide == 0) { // left
            gyroDrive(-35, 0.5, 48, 5.0);

            gyroHold(0.5);

            gyroDrive(-35, 0.5, 24, 3.0);

            gyroHold(0.5);

//            gyroDrive(-35, -0.5, 12, 3.0);
        } else if (goldSide == 2) { // right
            gyroDrive(45, 0.5, 48, 5.0);

            gyroHold(0.5);

            gyroDrive(35, 0.5, 24, 5.0);

//            gyroDrive(35, -0.5, 12, 3.0);
        } else {
            gyroDrive(0, 0.5, 24, 5.0);

            gyroHold(0.5);

//            gyroDrive(0, -0.5, 12, 3.0);
        }

        gyroHold(1);

        gyroDrive(0, 0.5, 50, 5.0);

        // turn to wall
        /*gyroHold(1);

        gyroDrive(90, 0.5, 36, 5.0);

        gyroHold(0.5);

        gyroDrive(90, -1, 24, 3.0);

        // drive to depot
        gyroDrive(135, -0.5, 36, 5.0);

        gyroHold(0.5);

        gyroDrive(135, -0.75, 36, 5.0);
        // push off second mineral

        // score team marker

        // navigate to crater

        measureGyroAccuracy(5.0);
        telemetry.addLine("Done measuring. Drive time!");
        gyroDrive(0, 0.5, 12, 3);
        telemetry.addLine("Turning.");
        gyroDrive(90, 0.5, 12, 3.0);*/

        detector.disable();
    }

    private void measureGyroAccuracy(double timeout) {
        double targetAngle = 0;

        double startTime = time;
        double[] iterationTimes = new double[100];

        double startLPos = wheel1.getCurrentPosition();
        double startRPos = wheel2.getCurrentPosition();
        double power = 0;
        int i = 0;
        for (i = 0; i <= 99; i++) {
            if (time - startTime > timeout) {
                telemetry.addData("last drive stop", time);
                break;
            }
            double loopStart = time;
            Orientation orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            // angles                  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double error = (targetAngle - (orientation.thirdAngle > 180 ? orientation.thirdAngle - 360 : orientation.thirdAngle));
            double correction = error * DRIVE_kP;
            //error = Range.clip(error, -1, 1);
            telemetry.addData("motorleft dist:", wheel2.getCurrentPosition() - startLPos);
            telemetry.addData("motorright dist:", wheel1.getCurrentPosition() - startRPos);
            telemetry.addData("Current z orientation:", orientation.thirdAngle);
            telemetry.addData("error", error);
            telemetry.update();
            double right = power + correction;
            double left = power - correction;

            double max = Math.max(Math.abs(right), Math.abs(left));

            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            iterationTimes[i] = time - loopStart;
        }

        double averageIterationTime = averageArray(iterationTimes);

        telemetry.addData("AVG TIME:", averageIterationTime);
        telemetry.update();
    }

    /**
     * TODO: Think about clipping the correction rather than final power
     *
     * @param targetAngle the desired angle to keep
     * @param desiredPower the preferred motor power
     * @param distance the distance, in inches, to drive
     * @param timeout the amount of time to elapse before aborting
     */
    private void gyroDrive(double targetAngle, double desiredPower, double distance, double timeout) {
        PController controller = new PController(1/20);

        distance = distance * TICKS_PER_INCH;

        double rightStart = bot.wheel1.getCurrentPosition(); // back right
        double leftStart = bot.wheel2.getCurrentPosition(); // back left

        double startTime = time;

        while (opModeIsActive() && Math.abs(bot.wheel1.getCurrentPosition() - rightStart) < distance && Math.abs(bot.wheel2.getCurrentPosition() - leftStart) < distance) {
            if (time > startTime + timeout) { // timeout
                telemetry.addLine("Drive loop timeout.");
                break;
            }

            Orientation orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            double angle = orientation.thirdAngle;
            // angles                  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double error = targetAngle - angle;
            double correction = error * 1/20; //controller.calculate(error);
            double rightPower = desiredPower - correction;
            double leftPower  = desiredPower + correction;

            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.addData("rightPower", rightPower);
            telemetry.addData("leftPower", leftPower);
            telemetry.addData("kP", controller.getkP());
            telemetry.addData("rightEncoder", bot.wheel1.getCurrentPosition());
            telemetry.addData("distance", distance);
            //telemetry.addData("Current angle:", orientation.thirdAngle);
            telemetry.update();

            double max = Math.max(Math.abs(rightPower), Math.abs(leftPower));
            if (max > 1) { // clip the power between -1,1 while retaining relative speed percentage
                rightPower = rightPower / max;
                leftPower = leftPower / max;
            }

            wheel1.setPower(rightPower);
            //wheel4.setPower(rightPower);
            wheel2.setPower(leftPower);
            //wheel3.setPower(leftPower);
        }

        wheel1.setPower(0);
        //wheel4.setPower(0);
        wheel2.setPower(0);
        //wheel3.setPower(0);
    }

    public void gyroHold(double holdTime) {
        double startTime = time;
        while (opModeIsActive() && time - startTime < holdTime) {
            Thread.yield();
        }
    }
    private void driveByGyro(double targetAngle, double power,  double distance, double timeout) {

        double encoderTicks = inchesToTicks(distance);
        // Set the motors to measure encoder counts.
        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int startRPos = wheel1.getCurrentPosition();
        int startLPos = wheel2.getCurrentPosition();
        double startTime = time;

        power = Range.clip(-power, -1, 1);
        // Start the loop. Even if we're going backwards, math.abs makes sure that we can still compare to distance.
        while (opModeIsActive() && (Math.abs(wheel2.getCurrentPosition() -startRPos) < distance) && (Math.abs(wheel1.getCurrentPosition() -startLPos) < distance)) {
            if (time > startTime + timeout) {
                telemetry.addData("Last drive stop:", "Timeout!");
                break;
            }

            Orientation orientation = bot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            // angles                  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double error = (targetAngle - (orientation.thirdAngle > 180 ? orientation.thirdAngle - 360 : orientation.thirdAngle));
            double correction = error * DRIVE_kP;
            //error = Range.clip(error, -1, 1);
            telemetry.addData("motorleft dist:", wheel2.getCurrentPosition() - startLPos);
            telemetry.addData("motorright dist:", wheel1.getCurrentPosition() - startRPos);
            telemetry.addData("Current z orientation:", orientation.thirdAngle);
            telemetry.addData("error", error);
            telemetry.update();
            double right = power + correction;
            double left = power - correction;

            double max = Math.max(Math.abs(right), Math.abs(left));

            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            wheel1.setPower(right);
            //wheel4.setPower(right);
            //wheel3.setPower(left);
            wheel2.setPower(left);
        }

        telemetry.addData("Last drive stop:", "Loop complete!");
        wheel1.setPower(0.0);
        wheel2.setPower(0.0);
        //wheel3.setPower(0.0);
        //wheel4.setPower(0.0);

        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //return (time < startTime + timeout);
    }

    private double averageArray(double[] vals) {
        double average = 0;

        for (int i = 0; i <= vals.length - 1; i++ ) {
            average = average + vals[i];
        }

        return average/vals.length;
    }
    private double inchesToTicks(double inches) {
        return inches * TICKS_PER_INCH;
    }
}
