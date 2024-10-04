package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extendo extends SubsystemBase {
    public static final double EXTEND_POS = 0;
    public static final double RETRACT_POS = 1;
    private final Servo m_extendo;

    public Extendo(final HardwareMap hwMap) {
        m_extendo = hwMap.get(Servo.class, "Extendo");
    }

    public void getPosition() {
        m_extendo.getPosition();
    }

    public void extend() {
        m_extendo.setPosition(EXTEND_POS);
    }

    public void retract() {
        m_extendo.setPosition(RETRACT_POS);
        m_extendo.setPosition(RETRACT_POS);
    }
}
