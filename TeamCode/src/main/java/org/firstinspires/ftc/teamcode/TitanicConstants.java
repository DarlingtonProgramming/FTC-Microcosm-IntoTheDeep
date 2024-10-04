package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class TitanicConstants {
    public enum AllianceType {
        BLUE, RED
    }
    public enum AllianceSide {
        CLOSE, FAR
    }

    public static boolean IS_COMPETITION = false;

    /*
     * These are the relative positions of each component from the center of the
     * robot (inches).
     */

    //
    public static RevHubOrientationOnRobot CONTROL_HUB_ORIENTATION = new RevHubOrientationOnRobot(xyzOrientation(180, 90, -21.9095));
}