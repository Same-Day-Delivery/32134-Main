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
    private DcMotor ShootL;
    private DcMotor ShootR;


    // Variables
    double shootPower = 0.8;
    double intakeSpeed = 1;



    // Don't Touch
    boolean shootState = false;
    boolean inState = false;

    boolean passState = false;






    @Override
    public void runOpMode() throws InterruptedException {


        Intake = hardwareMap.get(DcMotor.class, "Intake");
        ShootL = hardwareMap.get(DcMotor.class, "shootLeft");
        ShootR = hardwareMap.get(DcMotor.class, "shootRight");

        ShootR.setDirection(DcMotorSimple.Direction.REVERSE);
        ShootL.setDirection(DcMotorSimple.Direction.FORWARD);





        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(PoseStorage.currentPose);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        while (!isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();





            Vector2d input = new Vector2d(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
                    ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            gamepad1.right_stick_x
                    )
            );
            drive.update();




            // Shooter


            if (gamepad1.bWasPressed()) {
                shootState = !shootState;
            }

            if (shootState) {
                ShootL.setPower(shootPower);
                ShootR.setPower(shootPower);
                Intake.setPower(intakeSpeed);

            }
            else{
                ShootL.setPower(0.05);
                ShootR.setPower(0.05);
                Intake.setPower(0);
            }









            // Intake

            if (gamepad1.xWasPressed()) {
                inState = !inState;
            }

            if (inState){
                Intake.setPower(intakeSpeed);
            }
            else {
                Intake.setPower(0);
            }















            // Telemetry

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Inputs", gamepad1.toString());
            telemetry.update();
        }
        

    }
}
