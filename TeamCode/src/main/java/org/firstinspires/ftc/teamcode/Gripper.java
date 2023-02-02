package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    Servo m_servo;
    static final double grip_open=0.7;
    static final double grip_close=0.0;

    /** a psuedo timer; should use real time */
    int m_count = 0;

    enum GripState {
        GRIP_OPEN(0.8),
        GRIP_CLOSED(0.0);

        final double position;

        GripState(double pos) {
            position = pos;
        }
    }

    /**
     * Create the Gripper instance
     * @param hardwareMap
     */
    Gripper(HardwareMap hardwareMap){
        // get the servo
        m_servo = hardwareMap.get(Servo.class,"ServoAJ");

        // set the servo to open (needed to stay within the 18-inch cube
        grip(GripState.GRIP_OPEN);
    }

    /**
     * Set the gripper to a specific position.
     * @param position 0 to 1
     */
    public void setPosition(double position) {
        m_servo.setPosition(position);
        m_count = 200;
    }

    /**
     * Operate the gripper
     * @param gs
     */
    public void grip(GripState gs) {
        // set the position
        setPosition(gs.position);
    }

    /**
     * Operate the gripper.
     * @param state true means open, false means closed
     */
    public void grip(boolean state) {
        if (state)
            grip(GripState.GRIP_OPEN);
        else
            grip(GripState.GRIP_CLOSED);
    }

    /**
     * Has the gripper finished moving?
     * @return true if finished
     */
    public boolean finished() {
        if (m_count <= 0) {
            return true;
        }
        else {
            m_count--;
            return false;
        }
    }


}