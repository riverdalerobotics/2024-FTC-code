package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SecondRobotCode.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class BackAndForth extends LinearOpMode {


    public static double XDISTANCE = 20; //inches
    public static double YDISTANCE = 10; //inches
    public static double rotation = 90;


    public DcMotorEx motorLeftF;
    public DcMotorEx motorRightF;
    public DcMotorEx motorRightB;
    public DcMotorEx motorLeftB;
    IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private DcMotor armMotor;

    private DcMotor slideMotor;
    private Servo bucketServo;

    private Servo wristServo;
    private CRServo intakeServo;

    public WebcamName camera;

    ChassisSubsystem chassis;
    ArmSubsystem arm;
    IntakeSubsystem intake;
    SlidesSubsystem slides;

    Pose2d startingPose;
    Trajectory trajectoryForward;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        motorLeftF = hardwareMap.get(DcMotorEx.class ,"motorLeftF");
        motorRightF = hardwareMap.get(DcMotorEx.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotorEx.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotorEx.class, "motorLeftB");
        armMotor =hardwareMap.get(DcMotor.class, "armMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        bucketServo = hardwareMap.get(Servo.class, "bucket");
        wristServo = hardwareMap.get(Servo.class, "wrist");
        intakeServo = hardwareMap.get(CRServo.class, "intake");


        chassis = new ChassisSubsystem(motorLeftF, motorRightF, motorLeftB,motorRightB, imu,batteryVoltageSensor);
        arm = new ArmSubsystem(armMotor);
        intake = new IntakeSubsystem (intakeServo, wristServo);
        slides = new SlidesSubsystem(slideMotor, bucketServo);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        startingPose = new Pose2d(0, 0, Math.toRadians(0));

        Pose2d linePose = new Pose2d(XDISTANCE,YDISTANCE, Math.toRadians(90));

        chassis.setPoseEstimate(startingPose);

        trajectoryForward = chassis.trajectoryBuilder(startingPose)
                .forward(XDISTANCE)
                .build();



        Trajectory strafe = chassis.trajectoryBuilder(startingPose)
                .strafeLeft(70)
                .build();



        Trajectory gotoPose = chassis.trajectoryBuilder(startingPose)
                .lineToLinearHeading(
                        new Pose2d(XDISTANCE, YDISTANCE, rotation),
                        chassis.getVelocityConstraint(30, 200, Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(20)
                )
                .build();


        waitForStart();

        if(isStopRequested()) return;
        //slides.setHeight(10);

        telemetry.addData("pose", chassis.getPoseEstimate());
        chassis.followTrajectory(gotoPose);





        telemetry.update();



           // chassis.followTrajectory(trajectoryBackward);
//slides.setHeight(100);




        }
    }