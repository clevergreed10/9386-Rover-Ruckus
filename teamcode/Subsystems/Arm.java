package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.EEBotHardware;

public class Arm {

    private DcMotor armMotor;
    private DcMotor extensionMotor;


    private int currentArmPos;
    private int lastArmPos;
    private int targetArmPos;

    private int positionTargetA, positionTargetB;

    private double currentVelocity;
    private double previousVelocity;
    private double minVelocity;
    private double maxVelocity;

    private double currentAcceleration;
    private double previousAcceleration;
    private double minAcceleration;
    private double maxAcceleration;

    private double lastUpdateTime;

    public Arm(EEBotHardware bot) {
        this.armMotor = bot.armMotor;
        this.extensionMotor = bot.winch;
    }

    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    public void rotate(int position) {
        armMotor.setTargetPosition(position);
    }

    public void reset() {
        minVelocity = 0;
        maxVelocity = 0;
        minAcceleration = 0;
        maxAcceleration = 0;
    }

    public void update() {
        if (lastUpdateTime == 0) {
            lastUpdateTime = System.currentTimeMillis() / 1000;
            lastArmPos = armMotor.getCurrentPosition();
        } else {
            previousVelocity = currentVelocity;
            previousAcceleration = currentAcceleration;

            double currentTime = System.currentTimeMillis() / 1000;
            currentArmPos  = armMotor.getCurrentPosition();

            double deltaT = currentTime - lastUpdateTime;
            currentVelocity = (currentArmPos - lastArmPos)/deltaT;
            currentAcceleration = (currentVelocity - previousVelocity)/deltaT;

            maxVelocity = Math.max(maxVelocity, currentVelocity);
            minVelocity = Math.min(minVelocity, currentVelocity);
            maxAcceleration = Math.max(maxAcceleration, currentAcceleration);
            minAcceleration = Math.min(minAcceleration, currentAcceleration);
        }
    }

    public void setTargetA(int setpos) { positionTargetA = setpos; }

    public void setTargetB(int setpos) { positionTargetB = setpos; }

    public void setTarget(int target) {
        targetArmPos = target;

        // generate path/trajectory to target
    }

    // TODO: Make this dynamic (only one function for all presets)
    public void gotoTargetA() {
        setTarget(positionTargetA);
    }

    public void gotoTargetB() {
        setTarget(positionTargetB);
    }

    public int getArmPosition() {
        return currentArmPos;
    }

    public double getVelocity() {
        return currentVelocity;
    }

    public double getAcceleration() {
        return currentAcceleration;
    }

    public double getMaxVelocity() { return maxVelocity; }

    public double getMinVelocity() { return minVelocity; }

    public double getMaxAcceleration() { return maxAcceleration; }

    public double getMinAcceleration() { return minAcceleration; }
}
