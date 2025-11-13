package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoMainBlue extends LinearOpMode {
    // Devices
    private DcMotor Intake;
    private DcMotor Pass;
    private DcMotor Shoot;

    private ElapsedTime passTime = new ElapsedTime();

    // Variables
    double shootPower = 1;
    int passDist = 4;
    int passRes = 752;
    double nowPassTime = passTime.milliseconds();
    double maxPassTime = 50;
    double intakeSpeed = 1;
    long emptyTime = 1000;

    // BLEH

    // Don't Touch
    boolean shootState = false;
    boolean inState = false;
    double pass = 0;
    
    
    
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;
        Pose2d poseEstimate = drive.getPoseEstimate();


// shooting position
        Trajectory traj = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 0), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj);



// ball set 1 (top row)
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 0), Math.toRadians(270))
                .build();

        drive.followTrajectory(traj1);

// ball pickup set 1 (top row)
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 40), Math.toRadians(270))
                .build();

        drive.followTrajectory(traj2);

// shooting position
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 0), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj3);



// ball set 2 (middle row)
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(48, 0), Math.toRadians(270))
                .splineTo(new Vector2d(48, 60), Math.toRadians(270))
                .build();

        drive.followTrajectory(traj4);

// ball set 2 pickup (middle row)
        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 0), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj5);


// shooting position
        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 0), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj6);

    }
}
