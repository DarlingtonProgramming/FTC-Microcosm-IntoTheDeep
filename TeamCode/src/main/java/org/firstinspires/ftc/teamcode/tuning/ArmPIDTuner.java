package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class ArmPIDTuner extends LinearOpMode {
    public static double p = 0.013, i = 0, d = 0, f = 0;
    public static double targetPos = 0;
    public static double ticksToDeg = 2675 / 180;
    public static double manualArmPower = 1;
    private PIDController controller;
    private DcMotorEx arm;
    private boolean manualMode = false;

    @Override
    public void runOpMode() {
        Gamepad previousGamepad = new Gamepad();
        Gamepad currentGamepad = new Gamepad();

        controller = new PIDController(p, i, d);
        arm = hardwareMap.get(DcMotorEx.class, "LSlide");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            int currentPos = arm.getCurrentPosition();

            if (!manualMode) {
                controller.setPID(p, i, d);

                double pid = controller.calculate(currentPos, targetPos);
                // double ff = Math.cos(Math.toRadians(targetPos / ticksInDeg)) * f;

                double power = pid; // + ff

                arm.setPower(power);
            } else {
                if (currentGamepad.dpad_up) {
                    arm.setPower(manualArmPower);
                } else if (currentGamepad.dpad_down) {
                    arm.setPower(-manualArmPower);
                } else {
                    arm.setPower(0);
                }

                if (!previousGamepad.dpad_left && currentGamepad.dpad_left) {
                    manualArmPower = Range.clip(manualArmPower - 0.05, 0, 1);
                }

                if (!previousGamepad.dpad_right && currentGamepad.dpad_right) {
                    manualArmPower = Range.clip(manualArmPower + 0.05, 0, 1);
                }
            }

            if (!previousGamepad.a && currentGamepad.a) {
                manualMode = !manualMode;
            }

            telemetry.addData("Current Position (ticks)", currentPos);
            telemetry.addData("Target Position (ticks)", targetPos);
            telemetry.addData("Current Position (deg)", currentPos / ticksToDeg);
            telemetry.addData("Target Position (deg)", targetPos / ticksToDeg);
            telemetry.addData("Manual", manualMode);
            telemetry.addData("Manual Speed", manualArmPower);
            telemetry.update();
        }
    }
}
