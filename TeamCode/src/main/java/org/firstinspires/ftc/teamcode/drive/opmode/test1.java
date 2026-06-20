package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(group = "drive")
public class test1 extends LinearOpMode {

    private DcMotor Shoot;


    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        shoot = hardwareMap.get(DcMotor.class, "leftEncoder");

        waitForStart();

        if (isStopRequested()) return;

        Pose2d poseEstimate = drive.getPoseEstimate();

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))



    }




}
