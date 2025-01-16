package org.firstinspires.ftc.teamcode.RobotCode;
import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.RobotCode.ChassisSubsystem;
import com.acmerobotics.dashboard.FtcDashboard;

@Autonomous(name = "Emmanuel Auto Try 2", group = "Linear OpMode")
@Config
public class AutoII extends LinearOpMode{
    public static double distance = 2000;


    public double degToRotation(double deg) {
        return deg / 360 * 1425.2 * 5;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
//        FtcDashboard dashboard = FtcDashboard.getInstance;
//        MultipleTelematry telematryA= new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
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

        IMU imu = hardwareMap.get(IMU.class,"imu");
        ChassisSubsystem chassis = new ChassisSubsystem(left, right,imu);
        ArmSubsystem arm = new ArmSubsystem(armPivot);
        Commands commands = new Commands(armPivot, wrist, intakeServo);
        double armAngle = armPivot.getCurrentPosition() * 360 / (1425.2 * 5);
        armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(0.1);
        armPivot.setPower(0.8);


        waitForStart();

        if (isStopRequested()) return;

        //TODO: try if loop
        if(opModeIsActive()) {

            while(left.getCurrentPosition()>-distance){

                left.setPower(0.1);
                right.setPower(0.1);
                left.setTargetPosition(10000);
                right.setTargetPosition(10000);
                //chassis.drive(0.1,0);
            }

            if (left.getCurrentPosition()<-(distance+(distance/2))) {

                    chassis.drive(0, 0);
                    wrist.setPosition(0.45);
                    armPivot.setPower(0.9);
                    armPivot.setTargetPosition((int) degToRotation(183));
                    armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            armPivot.setPower(0.89);
            armPivot.setTargetPosition((int) degToRotation(159.0));
            armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);



//            telemetryA.addData("left wheel value", left.getCurrentPosition());
//            telemetryA.addData("right wheel value", right.getCurrentPosition());
//            telemetryA.addData("YAW", chassis.getYaw());
//            telemetryA.update();
            }
        }
    }

