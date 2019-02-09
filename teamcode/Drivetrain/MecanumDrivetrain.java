package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by Josh on 12-11-2018
 *
 * Program Details
 *      The Drivetrain class to use  when the robot has 4 mecanum wheels equipped.
 *      Remember to ensure that mecanum wheels are in the X configuration (wheel spinners make X shape from top-down)
 *
 * When instantiating: wheel order should be [Front_Right, Front_Left, Back_Left, Back_Right]
 */
public class MecanumDrivetrain extends Drivetrain {

    private int numWheels = 4;

    BNO055IMU imu;

    // consider adding MecanumDrivetrain(DcMotor frontLeft, frontRight, etc.)
    public MecanumDrivetrain(DcMotor[] wheels) {
        super(wheels);
    }

    public MecanumDrivetrain(DcMotor[] wheels, BNO055IMU gyro) {
        super(wheels);
        imu = gyro;
    }


    /**
     * Use this to drive in a specific direction relative to the robot
     * You will be unable to rotate using this method
     * @param direction: the angle in which to drive. 0 is forward, 90 is right, 180 is backwards, -90 is left
     * @param power: The power to supply to the motors: from 0 to 1
     */
    public void drive(double direction, double power) {
        double forward = Math.cos(Math.toRadians(direction)) * power;
        double strafe = Math.sin(Math.toRadians(direction)) * power;

        drive(forward, strafe, 0);
    }

    /**
     * Use this for precise
     * @param forward: the forward movement value. Ranges from -1 to 1
     * @param strafe:  the strafing movement value. Ranges from -1 to 1. 1 is right
     * @param rotate:  the rotation value. Ranges from -1 to 1. 1 should be clockwise
     */
    public void drive(double forward, double strafe, double rotate) {

        double frontLeft  = forward + rotate + strafe;
        double frontRight = forward - rotate - strafe;
        double backLeft   = forward + rotate - strafe;
        double backRight  = forward - rotate + strafe;

        //normalize speeds to be between -1 and 1
        double max = Math.abs(frontLeft);
        if (Math.abs(frontRight) > max) max = Math.abs(frontRight);
        if (Math.abs(backLeft) > max) max = Math.abs(backLeft);
        if (Math.abs(backRight) > max) max = Math.abs(backRight);

        if (max > 1) {
            frontLeft  /= max;
            frontRight /= max;
            backLeft   /= max;
            backRight  /= max;
        }

        wheels[0].setPower(frontRight);
        wheels[1].setPower(frontLeft);
        wheels[2].setPower(backLeft);
        wheels[3].setPower(backRight);
    }

    public void drive(double forward, double strafe, double rotate, boolean relativeToGyro) {
        if (relativeToGyro) {
            // adjust forward/strafe

            double angle = Math.toRadians(imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            // if gyro angle is clockwise then:
            /*double temp = forward*Math.cos(angle) + strafe*Math.sin(angle);
            strafe = -forward*Math.sin(angle) + strafe*Math.cos(angle);
            forward = temp;*/

            //otherwise...
            /*double temp = forward*Math.cos(angle) - strafe*Math.sin(angle);
            strafe = forward*Math.sin(angle) + strafe*Math.cos(angle);
            forward = temp;*/

            double temp = forward*Math.cos(angle) - strafe*Math.sin(angle);
            strafe      = forward*Math.sin(angle) + strafe*Math.cos(angle);
            forward     = temp;
        }

        drive(forward, strafe, rotate);
    }

    public void setGyroReference(double referenceAngle) {

    }
}
