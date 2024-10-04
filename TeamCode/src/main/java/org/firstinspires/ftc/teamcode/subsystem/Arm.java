package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class Arm extends SubsystemBase {
    public static PIDCoefficients PID = new PIDCoefficients(0.013, 0, 0);
    public static int LIFT_TOLERANCE = 50;
    private final DcMotorEx m_arm;

    public Arm(final HardwareMap hwMap) {
        m_arm = hwMap.get(DcMotorEx.class, "LSlide");
        m_arm.setDirection(DcMotorSimple.Direction.REVERSE);
        m_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_arm.setPower(0);
    }

    public void resetPosition() {
        m_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getPosition() {
        return m_arm.getCurrentPosition();
    }

    public void setPower(double power) {
        m_arm.setPower(power);
    }

    public boolean isWithinTolerance(double target) {
        return Math.abs(target - getPosition()) <= LIFT_TOLERANCE;
    }
}