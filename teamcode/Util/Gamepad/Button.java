package org.firstinspires.ftc.teamcode.Util.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Josh on 10-16-2018
 * Class to handle button interactions and logic
 */
public class Button {

    boolean down = false;
    boolean toggle = false;

    double downAt;
    double lastTime;

    private boolean wasDown = false;

    // Constructor
    public Button() {

    }


    // call this every teleop loop with
    // example: aButton.update(gamepad1.a)
    public void update(boolean down) {
        update(down, 0);
    }

    public void update(boolean down, double currentTime) {
        // do logic to calculate toggle and other stuff
        this.down = down;
        this.lastTime = currentTime;

        if (down != this.wasDown) {
            if (down == false) {
                this.toggle = !this.toggle;
            }
            this.wasDown = down;
        }
        if (this.down && this.downAt == 0) {
            this.downAt = currentTime;
        } else if (!this.down) {
            this.downAt = 0;
        }

    }


    public boolean isDown() { // to determine if a button is down
        return this.down;
    }

    public double getDownTime () {
        if (this.downAt == 0) {
            return 0;
        }
        else {
            return this.lastTime - this.downAt;
        }
    }

    public boolean isToggle() {
        return this.toggle;
    }

}