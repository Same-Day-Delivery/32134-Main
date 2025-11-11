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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(group = "drive")
public class TeleopMainBlue extends LinearOpMode {

    // Devices
    private DcMotor Intake;
    private DcMotor Pass;
    private DcMotor Shoot;
    private VisionPortal visionPortal;
    private ElapsedTime passTime = new ElapsedTime();


    // Variables
    double shootPower = 1;
    int passDist = 4;
    int passRes = 752;
    double nowPassTime = passTime.milliseconds();
    double maxPassTime = 50;
    double intakeSpeed = 1;
    double trackSpeed = 5;



    // Don't Touch
    boolean shootState = false;
    boolean inState = false;
    boolean trackState = false;
    double pass = 0;
    double offset = 0;
    double offsetX;
    double offsetY;
    double offsetZ;





    @Override
    public void runOpMode() throws InterruptedException {

        Intake = hardwareMap.get(DcMotor.class, "frontEncoder");
        Pass = hardwareMap.get(DcMotor.class, "INTAKE");
        Shoot = hardwareMap.get(DcMotor.class, "rightEncoder");


        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        while (!isStopRequested()) {

            if(gamepad2.rightBumperWasPressed()) {
                trackState = !trackState;
            }

            if(trackState){
                offset = offsetX/trackSpeed;
            }

            offset = offsetX/trackSpeed;


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y - gamepad2.left_stick_y/10,
                            -gamepad1.left_stick_x - gamepad2.left_stick_x/10,
                            -gamepad1.right_stick_x - gamepad2.right_stick_x/10 + offset
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();


            // Shooter
            if (gamepad2.leftBumperWasPressed()) {
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

            if (gamepad2.aWasPressed()) {
                pass += 1;
            }
            if (gamepad2.yWasPressed()) {
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


            // Camera
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();



            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == 20) {
                    offsetX = detection.rawPose.x;
                    offsetY = detection.rawPose.y;
                    offsetZ = detection.rawPose.z;
                }
            }



            // Telemetry

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Shoot Queue:", pass);
            telemetry.addData("Inputs", gamepad1.toString());
            telemetry.update();
        }
        visionPortal.close();
    }
}
