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

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}
