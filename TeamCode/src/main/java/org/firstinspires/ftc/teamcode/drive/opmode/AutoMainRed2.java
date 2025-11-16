package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoMainRed2 extends LinearOpMode {
    // Devices
    private DcMotor Intake;
    private CRServo Pass1;
    private CRServo Pass2;
    private DcMotor Shoot;

    private ElapsedTime passTime = new ElapsedTime();

    // Variables
    double shootPower = 0.8;
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

        Intake = hardwareMap.get(DcMotor.class, "rightEncoder");
        Shoot = hardwareMap.get(DcMotor.class, "leftEncoder");
        Pass1 = hardwareMap.get(CRServo.class, "Pass1");
        Pass2 = hardwareMap.get(CRServo.class, "Pass2");


        waitForStart();

        if (isStopRequested()) return;
        Pose2d poseEstimate = drive.getPoseEstimate();

        drive.setPoseEstimate(new Pose2d(84, 9, Math.toRadians(0)));



        Shoot.setPower(shootPower);
        Intake.setPower(intakeSpeed);

// shooting position
        Trajectory traj = drive.trajectoryBuilder(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getHeading()))
                .back(30)
                .build();
        drive.followTrajectory(traj);



        PoseStorage.currentPose = drive.getPoseEstimate();

    }
}
