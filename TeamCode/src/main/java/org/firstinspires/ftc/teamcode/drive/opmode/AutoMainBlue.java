package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoMainBlue extends LinearOpMode {
    // Devices
    private DcMotor Intake;
    private DcMotor ShootL;

    private DcMotor ShootR;

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

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        ShootL = hardwareMap.get(DcMotor.class, "shootLeft");
        ShootR = hardwareMap.get(DcMotor.class, "shootRight");

        ShootL.setDirection(DcMotorSimple.Direction.REVERSE);
        ShootL.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        if (isStopRequested()) return;
        Pose2d poseEstimate = drive.getPoseEstimate();

        drive.setPoseEstimate(new Pose2d(60, 9, Math.toRadians(0)));



        ShootL.setPower(shootPower);
        ShootR.setPower(shootPower);
        Intake.setPower(intakeSpeed);

        wait(2000);

        ShootR.setPower(0);
        ShootL.setPower(0);
        Intake.setPower(0);

// shooting position


    }
}
