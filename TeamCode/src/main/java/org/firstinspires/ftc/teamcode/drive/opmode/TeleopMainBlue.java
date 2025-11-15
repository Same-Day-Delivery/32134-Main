package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(group = "drive")
public class TeleopMainBlue extends LinearOpMode {

    // Devices
    private DcMotor Intake;
    private CRServo Pass1;
    private CRServo Pass2;
    private DcMotor Shoot;
    private VisionPortal visionPortal;


    // Variables
    double shootPower = 0.8;
    int passDist = 4;
    int passRes = 752;
    double maxPassTime = 50;
    double intakeSpeed = 1;
    double trackSpeed = 300;



    // Don't Touch
    boolean shootState = false;
    boolean inState = false;
    boolean trackState = false;
    boolean passState = false;
    boolean Cam = false;
    double pass = 0;
    double offset = 0;
    double offsetX = 0;
    double offsetY;
    double offsetZ;





    @Override
    public void runOpMode() throws InterruptedException {


        Intake = hardwareMap.get(DcMotor.class, "rightEncoder");
        Shoot = hardwareMap.get(DcMotor.class, "leftEncoder");



        Pass1 = hardwareMap.get(CRServo.class, "Pass1");
        Pass2 = hardwareMap.get(CRServo.class, "Pass2");
        Pass1.setDirection(DcMotorSimple.Direction.REVERSE);
        Pass2.setDirection(DcMotorSimple.Direction.REVERSE);



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

        drive.setPoseEstimate(PoseStorage.currentPose);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        while (!isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();

            if(gamepad1.dpadUpWasPressed()) {
                trackState = !trackState;
            }

            if(trackState){
                offset = offsetX / trackSpeed;
            }
            else {
                offset = 0;
            }


            if(gamepad1.left_stick_button) {
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ).rotated(-poseEstimate.getHeading());

                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x
                        )
                );
            }
            else {

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y - gamepad2.left_stick_y / 10,
                                -gamepad1.left_stick_x - gamepad2.left_stick_x / 10,
                                -gamepad1.right_stick_x - gamepad2.right_stick_x / 10 + offset
                        )
                );
            }
            drive.update();




            // Shooter


            if (gamepad1.leftBumperWasPressed()) {
                shootState = !shootState;
            }

            if (shootState) {
                Shoot.setPower(shootPower);

            }
            else{
                Shoot.setPower(0);
            }




            // Passthrough

            if(gamepad1.aWasPressed()){
                passState = !passState;
            }

            if(passState){
                Pass1.setPower(0.5);
            }
            else {
                Pass1.setPower(0);
            }


            if(gamepad1.bWasPressed()){
                passState = !passState;
            }

            if(passState){
                Pass2.setPower(0.5);
            }
            else {
                Pass2.setPower(0);
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
                Cam = true;
                if (detection.id == 20) {
                    offsetX = detection.rawPose.x;
                    offsetY = detection.rawPose.y;
                    offsetZ = detection.rawPose.z;
                }
                else{
                    offsetX = 0;
                    offsetY = 0;
                    offsetZ = 0;
                }
            }
            if (!Cam){
                offsetX = 0;
                offsetY = 0;
                offsetZ = 0;
            }
            else{
                Cam = false;
            }







            // Telemetry

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("camera x", offsetX);
            telemetry.addData("camera y", offsetY);
            telemetry.addData("camera speed", offset);
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Shoot Queue:", pass);
            telemetry.addData("Inputs", gamepad1.toString());
            telemetry.update();
        }

        visionPortal.close();
        

    }
}
