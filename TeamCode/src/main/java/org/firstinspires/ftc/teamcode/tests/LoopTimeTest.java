package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCore;

@Config
@TeleOp
public class LoopTimeTest extends CommandOpMode {
    private RobotCore robot;
    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot = new RobotCore(hardwareMap);
        robot.m_drive.startTeleopDrive();

        while (opModeInInit()) {
            telemetry.addLine("Robot initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        robot.m_drive.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        robot.m_drive.update();

        telemetry.addData("hz", elapsedTime.milliseconds());
        telemetry.update();
        elapsedTime.reset();
    }
}
