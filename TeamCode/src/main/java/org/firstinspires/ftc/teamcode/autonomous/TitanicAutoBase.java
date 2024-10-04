package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TitanicConstants;
import org.firstinspires.ftc.teamcode.TitanicCore;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public abstract class TitanicAutoBase extends CommandOpMode {
    protected final TitanicConstants.AllianceType m_allianceType;
    protected final TitanicConstants.AllianceSide m_allianceSide;
    protected final Pose2d m_startingPose;
    public TitanicCore robot;
    public WebcamName webcam1, webcam2;
    public VisionPortal vision;

    public TitanicAutoBase(TitanicConstants.AllianceType allianceType, TitanicConstants.AllianceSide allianceSide, Pose2d startPose) {
        m_allianceType = allianceType;
        m_allianceSide = allianceSide;
        m_startingPose = startPose;
    }

    @Override
    public void initialize() {
        robot = new TitanicCore(hardwareMap, m_allianceType, m_allianceSide, m_startingPose);
//        new InitPositions(robot.m_lift, robot.m_outtake, robot.m_intake).schedule();

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        while (vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for cameras to be ready...");
            telemetry.update();
            sleep(10);
        }

        vision.setActiveCamera(webcam2);

        telemetry.addLine("Initializing trajectories...");
        telemetry.update();

        initAuto();
        startAuto();
    }

    @Override
    public void run() {
        super.run();
        robot.m_drive.updatePoseEstimate();
        PoseStorage.currentPose = robot.m_drive.pose;
    }

    public abstract void initAuto();
    public abstract void startAuto();
}