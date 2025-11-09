package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class TeleopMain extends LinearOpMode {

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

        Intake = hardwareMap.get(DcMotor.class, "frontEncoder");
        Pass = hardwareMap.get(DcMotor.class, "INTAKE");
        Shoot = hardwareMap.get(DcMotor.class, "rightEncoder");



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();


            // Shooter
            if (gamepad1.leftBumperWasPressed()) {
                shootState = !shootState;
            }

            if (shootState) {
                Shoot.setPower(shootPower);
            }
            else {
                Shoot.setPower(0);
            }

            // Passthrough
            nowPassTime = passTime.milliseconds();

            if (gamepad1.aWasPressed()) {
                pass += 1;
            }
            if (gamepad1.yWasPressed()) {
                pass += 5;
            }

            if (pass > 0 && nowPassTime>maxPassTime) {
                Pass.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Pass.setTargetPosition(passRes/passDist);
                Pass.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Pass.setPower(1);
                while (Pass.isBusy()){
                    // keep running
                }
                Pass.setPower(0);
                Pass.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pass -= 1;
                passTime.reset();
            }

            // Intake

            if (gamepad1.rightBumperWasPressed()) {
                inState = !inState;
            }

            if (inState){
                Intake.setPower(intakeSpeed);
            }
            else {
                Intake.setPower(0);
            }





            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Shoot Queue:", pass);
            telemetry.addData("Inputs", gamepad1.toString());
            telemetry.update();
        }
    }
}
