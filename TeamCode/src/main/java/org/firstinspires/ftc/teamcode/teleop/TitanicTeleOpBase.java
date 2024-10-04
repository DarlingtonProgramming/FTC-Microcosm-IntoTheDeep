package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.TitanicConstants;
import org.firstinspires.ftc.teamcode.TitanicCore;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

public abstract class TitanicTeleOpBase extends CommandOpMode {
    private final TitanicConstants.AllianceType m_allianceType;
    private final TitanicConstants.AllianceSide m_allianceSide;

    protected TitanicCore robot;
    protected IMU imu;

    public TitanicTeleOpBase(TitanicConstants.AllianceType allianceType, TitanicConstants.AllianceSide allianceSide) {
        m_allianceType = allianceType;
        m_allianceSide = allianceSide;
    }

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new TitanicCore(hardwareMap, m_allianceType, m_allianceSide, PoseStorage.currentPose);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(TitanicConstants.CONTROL_HUB_ORIENTATION);
        imu.initialize(parameters);

//        new InitPositions(robot.m_lift, robot.m_outtake, robot.m_intake).schedule();

        initTeleOp();
    }

    protected void driveFieldCentric(double leftStickX, double leftStickY, double rightStickX, double speed) {
        double odometryAngle = robot.m_drive.pose.heading.toDouble();

        double x = -leftStickY;
        double y = -leftStickX;

        double rotatedX = x * Math.cos(-odometryAngle) - y * Math.sin(-odometryAngle);
        double rotatedY = x * Math.sin(-odometryAngle) + y * Math.cos(-odometryAngle);

        robot.m_drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotatedX, rotatedY).times(speed), -rightStickX));
    }

    protected void driveFieldCentric(double leftStickX, double leftStickY, double rightStickX, double gyroAngle, double speed) {
        double x = -leftStickY;
        double y = -leftStickX;

        double rotatedX = x * Math.cos(-gyroAngle) - y * Math.sin(-gyroAngle);
        double rotatedY = x * Math.sin(-gyroAngle) + y * Math.cos(-gyroAngle);

        robot.m_drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotatedX, rotatedY).times(speed), -rightStickX));
    }

    protected void driveRobotCentric(double leftStickX, double leftStickY, double rightStickX) {
        robot.m_drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-leftStickY, -leftStickX), -rightStickX));
    }

    protected void driveRobotCentric(double leftStickX, double leftStickY, double rightStickX, double speed) {
        robot.m_drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-leftStickY, -leftStickX).times(speed), -rightStickX * speed));
    }

    public abstract void initTeleOp();
}