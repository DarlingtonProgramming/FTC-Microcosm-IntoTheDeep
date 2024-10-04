package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends SubsystemBase {
    public static int LIFT_TOLERANCE = 50;
    private final DcMotorEx m_lift;

    public Lift(final HardwareMap hwMap) {
        m_lift = hwMap.get(DcMotorEx.class, "Lift");
        m_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        m_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getPosition() {
        return m_lift.getCurrentPosition();
    }

    public double getVelocity() {
        return m_lift.getVelocity();
    }

    public void setPosition(int pos) {
        m_lift.setTargetPosition(pos);
    }

    public void setPower(double power) {
        m_lift.setPower(power);
    }

    public boolean isWithinTolerance(double target) {
        return Math.abs(target - m_lift.getCurrentPosition()) <= LIFT_TOLERANCE;
    }
}