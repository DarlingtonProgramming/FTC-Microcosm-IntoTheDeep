package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class BlueRight extends CommandOpMode {
    public static final Pose startingPose = new Pose(-27.4, 60, Math.toRadians(180));
    private RobotCore robot;
    public PathChain path;

    @Override
    public void initialize() {
        robot = new RobotCore(hardwareMap);
        robot.m_drive.setStartingPose(startingPose);

        path = robot.m_drive.pathBuilder()
                .addPath(new BezierLine(new Point(startingPose), new Point(-24, 27)))
                .setConstantHeadingInterpolation(startingPose.getHeading())

                .addPath(new BezierLine(new Point(-24, 27), new Point(-25,32)))
                .setConstantHeadingInterpolation(startingPose.getHeading())

                .addPath(new BezierLine(new Point(-25,32), new Point(-32,32)))
                .setLinearHeadingInterpolation(startingPose.getHeading(), Math.PI)
                .build();

        schedule(
            new SequentialCommandGroup(
                new FollowPath(robot.m_drive, path)
            )
        );
    }

    @Override
    public void run() {
        super.run();
        robot.m_drive.update();
        telemetry.update();
    }
}
