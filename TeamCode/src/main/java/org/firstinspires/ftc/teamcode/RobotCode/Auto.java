package org.firstinspires.ftc.teamcode.RobotCode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math.*;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Emmanuel Auto", group = "Linear OpMode")
public class Auto extends LinearOpMode{

    public double degToRotation(double deg) {
        return deg / 360 * 1425.2 * 5;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        DcMotor left;
        DcMotor right;
        DcMotor armPivot;
        Servo wrist;
        CRServo intakeServo;
        left = hardwareMap.get(DcMotor.class, "LeftMotor");
        right = hardwareMap.get(DcMotor.class, "RightMotor");
        armPivot = hardwareMap.get(DcMotor.class, "ArmPivot");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
        ChassisSubsystem chassis = new ChassisSubsystem(left, right);
        ArmSubsystem arm = new ArmSubsystem(armPivot);
        Commands commands = new Commands(armPivot, wrist, intakeServo);
        OI oi = new OI(gamepad1, gamepad2);
        double armAngle = armPivot.getCurrentPosition() * 360 / (1425.2 * 5);
        armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean moveServo = false;
        boolean reverseDrive = false;
        wrist.setPosition(0.1);
        armPivot.setPower(0.8);
        armPivot.setTargetPosition((int) degToRotation(10));
        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (opModeIsActive()) {

            left.setDirection(DcMotor.Direction.REVERSE);
            right.setDirection(DcMotor.Direction.REVERSE);


            while (left.getCurrentPosition() > -3000 || right.getCurrentPosition() < 3000) {
                right.setTargetPosition(300);
                left.setTargetPosition(300);
                right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right.setPower(0.2);
                left.setPower(0.2);

                armPivot.setPower(0.8);
                armPivot.setTargetPosition((int) degToRotation(158.0));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                wrist.setPosition(0.45);

                armPivot.setPower(0.6);
                armPivot.setTargetPosition((int) degToRotation(190));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeServo.setPower(-0.8);

            }


            armPivot.setPower(0.8);
            armPivot.setTargetPosition((int) degToRotation(158.0));
            armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            wrist.setPosition(0.45);

            armPivot.setTargetPosition((int) degToRotation(190));
            armPivot.setPower(0.6);
            armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeServo.setPower(-0.8);

            telemetry.addData("left wheel value", left.getCurrentPosition());
            telemetry.addData("right wheel value", right.getCurrentPosition());
            telemetry.update();
        }
    }

    }
