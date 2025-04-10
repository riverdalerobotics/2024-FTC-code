package org.firstinspires.ftc.teamcode.RobotCode;
//import com.acmerobotics.dashboard.config.Config;
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
//import com.acmerobotics.dashboard.FtcDashboard;

@Autonomous(name = "KityKat park auto", group = "Linear OpMode")
//@Config
public class Auto extends LinearOpMode{
    public static double distance = 500;

    public static int part1 = 1;



    public double degToRotation(double deg) {
        return deg / 360 * 1425.2 * 5;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
//        FtcDashboard dashboard = FtcDashboard.getInstance;
//        MultipleTelematry telematryA=new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
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

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(0.1);
        armPivot.setPower(0.8);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("left wheel value", left.getCurrentPosition());
            telemetry.addData("right wheel value", right.getCurrentPosition());
            telemetry.addData("arm", armPivot);
            //telemetryA.addData("YAW", chassis.getYaw());
            telemetry.update();


            if (part1 == 1) {

                chassis.drive(0,-1);
                Thread.sleep(2000);
                chassis.drive(0.5,0);
                Thread.sleep(4000);
                chassis.drive(-1,0);
                Thread.sleep(10);

            }










        }
    }
}
