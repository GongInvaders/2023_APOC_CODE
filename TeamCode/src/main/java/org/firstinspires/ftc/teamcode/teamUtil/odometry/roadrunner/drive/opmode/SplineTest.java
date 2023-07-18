package org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                /*.splineTo(new Vector2d(40,0), Math.toRadians(0))
                .splineTo(new Vector2d(52, -12), Math.toRadians(-90))
                .splineTo(new Vector2d(40, -24), Math.toRadians(-180))
                .splineTo(new Vector2d(0,-24), Math.toRadians(-180))
                .splineTo(new Vector2d(-12, -12), Math.toRadians(-270))
                .splineTo(new Vector2d(0,0),0)*/

                /*.splineToSplineHeading(new Pose2d(40,0,Math.toRadians(15)),Math.toRadians(0))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(0)))
                .waitSeconds(0)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(40,0,Math.toRadians(15)),Math.toRadians(0))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(0)))
                .waitSeconds(0)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(40,0,Math.toRadians(15)),Math.toRadians(0))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(0)))
                .waitSeconds(0)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(40,0,Math.toRadians(15)),Math.toRadians(0))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(0)))
                .waitSeconds(0)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(40,0,Math.toRadians(15)),Math.toRadians(0))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(0)))
                .waitSeconds(0)
                .setReversed(false)*/
/*
                .splineTo(new Vector2d(6,0), 0)
                .splineTo(new Vector2d(50,-20), 0)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(180))
                .splineTo(new Vector2d(0,0), Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(6,0), 0)
                .splineTo(new Vector2d(50,-20), 0)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(180))
                .splineTo(new Vector2d(0,0), Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(6,0), 0)
                .splineTo(new Vector2d(50,-20), 0)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(180))
                .splineTo(new Vector2d(0,0), Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(6,0), 0)
                .splineTo(new Vector2d(50,-20), 0)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(180))
                .splineTo(new Vector2d(0,0), Math.toRadians(180))*/

                .splineTo(new Vector2d(40,0),0)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(45, -20), Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(40,0),Math.toRadians(90))
                .setReversed(false)
                .splineTo(new Vector2d(45, -20), Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(40,0),Math.toRadians(90))
                .setReversed(false)
                .splineTo(new Vector2d(45, -20), Math.toRadians(-90))
                .setReversed(true)
                .splineTo(new Vector2d(40,0),Math.toRadians(90))
                .setReversed(false)



                .build();

        drive.followTrajectorySequence(traj);

        sleep(1000);

    }
}
