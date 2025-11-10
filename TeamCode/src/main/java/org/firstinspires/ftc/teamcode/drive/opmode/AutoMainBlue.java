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

        Intake = hardwareMap.get(DcMotor.class, "frontEncoder");
        Pass = hardwareMap.get(DcMotor.class, "INTAKE");
        Shoot = hardwareMap.get(DcMotor.class, "rightEncoder");

        waitForStart();

        if (isStopRequested()) return;
        Pose2d poseEstimate = drive.getPoseEstimate();

        Shoot.setPower(shootPower);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 0), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj);

        Pass.setPower(1);
        sleep(emptyTime);
        Pass.setPower(0);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 0), Math.toRadians(270))
                .build();

        drive.followTrajectory(traj1);

        Intake.setPower(1);

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 40), Math.toRadians(270))
                .build();

        drive.followTrajectory(traj2);


        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 0), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj3);

        Pass.setPower(1);
        sleep(emptyTime);
        Pass.setPower(0);
        Intake.setPower(1);

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(48, 0), Math.toRadians(270))
                .splineTo(new Vector2d(48, 60), Math.toRadians(270))
                .build();

        drive.followTrajectory(traj4);

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .splineTo(new Vector2d(72, 0), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj4);

        Pass.setPower(1);
        sleep(emptyTime);
        Pass.setPower(0);
        Intake.setPower(0);
        Shoot.setPower(0);

    }
}
