package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.BulkCacheHandler;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

public class RobotCore extends Robot {
    // Subsystems
    public Follower m_drive;

    public RobotCore(HardwareMap hardwareMap) {
        // Schedule to clear cache continuously (manual mode)
        schedule(new BulkCacheHandler(hardwareMap));

        // Initialize subsystems
        m_drive = new Follower(hardwareMap);
    }
}