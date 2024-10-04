package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    public enum RotatePosition {
        DOWN,
        UP
    }
    public static final double GRAB_POS = 0;
    public static final double RETRACT_POS = 1;
    public static final double DOWN_POS = 0;
    public static final double UP_POS = 1;
    private final Servo m_grabber;
    private final Servo m_swivel;

    public Intake(final HardwareMap hwMap) {
        m_grabber = hwMap.get(Servo.class, "Grabber");
        m_swivel = hwMap.get(Servo.class, "Swivel");
    }

    public void getGrabberPosition() {
        m_grabber.getPosition();
    }

    public void grab() {
        m_grabber.setPosition(GRAB_POS);
    }

    public void retract() {
        m_grabber.setPosition(RETRACT_POS);
        m_grabber.setPosition(RETRACT_POS);
    }

    public void rotate(RotatePosition pos) {
        m_swivel.setPosition(pos == RotatePosition.DOWN ? DOWN_POS : UP_POS);
    }
}
