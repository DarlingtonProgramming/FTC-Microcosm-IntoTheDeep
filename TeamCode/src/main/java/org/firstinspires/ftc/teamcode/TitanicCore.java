package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.BulkCacheHandler;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class TitanicCore extends Robot {
    public final TitanicConstants.AllianceType ALLIANCE_TYPE;
    public final TitanicConstants.AllianceSide ALLIANCE_SIDE;

    // Subsystems
    public MecanumDrive m_drive;
    public Intake m_intake;
    public Extendo m_extendo;
    public Lift m_lift;

    public TitanicCore(HardwareMap hardwareMap, TitanicConstants.AllianceType allianceType, TitanicConstants.AllianceSide allianceSide, Pose2d startPose) {
        ALLIANCE_TYPE = allianceType;
        ALLIANCE_SIDE = allianceSide;

        // Schedule to clear cache continuously (manual mode)
        schedule(new BulkCacheHandler(hardwareMap));

        // Initialize subsystems
        m_drive = new MecanumDrive(hardwareMap, startPose);
        m_intake = new Intake(hardwareMap);
        m_extendo = new Extendo(hardwareMap);
        m_lift = new Lift(hardwareMap);
    }
}