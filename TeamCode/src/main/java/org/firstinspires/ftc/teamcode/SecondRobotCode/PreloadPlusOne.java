package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


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
public class PreloadPlusOne extends LinearOpMode {


    public DcMotorEx motorLeftF;
    public DcMotorEx motorRightF;
    public DcMotorEx motorRightB;
    public DcMotorEx motorLeftB;
    IMU imu;

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

    @Override
    public void runOpMode() throws InterruptedException {
        //8.65 inches length
        //8.9 inches wide

        chassis = new ChassisSubsystem(motorLeftF, motorRightF, motorLeftB,motorRightB,imu);
        arm = new ArmSubsystem(armMotor);
        intake = new IntakeSubsystem (intakeServo, wristServo);
        slides = new SlidesSubsystem(slideMotor, bucketServo);

        motorLeftF = hardwareMap.get(DcMotorEx.class ,"motorLeftF");
        motorRightF = hardwareMap.get(DcMotorEx.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotorEx.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotorEx.class, "motorLeftB");
        armMotor =hardwareMap.get(DcMotor.class, "armMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        bucketServo = hardwareMap.get(Servo.class, "bucket");
        wristServo = hardwareMap.get(Servo.class, "wrist");
        intakeServo = hardwareMap.get(CRServo.class, "intake");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        startingPose = new Pose2d(-30, 8.7, Math.toRadians(0));

        Trajectory trajectoryOneReposition = chassis.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();
        Trajectory lineUpBasketTrajectoryTwo = chassis.trajectoryBuilder(trajectoryOneReposition.end())
                .lineToLinearHeading(new Pose2d(-56, 16, Math.toRadians(-45)))
                .build();
        Trajectory lineupMiddleSampleTrajectoryThree = chassis.trajectoryBuilder(lineUpBasketTrajectoryTwo.end())
                .lineToLinearHeading(new Pose2d(-60, 30, Math.toRadians(0)))
                .build();
        Trajectory goIntakeSampleTrajectoryFour = chassis.trajectoryBuilder(lineupMiddleSampleTrajectoryThree.end())
                .lineToLinearHeading(new Pose2d(-60, 40, Math.toRadians(0)))
                .build();
        Trajectory lineupBasketTrajectoryFive = chassis.trajectoryBuilder(goIntakeSampleTrajectoryFour.end())
                .lineToLinearHeading(new Pose2d(-56, 16, Math.toRadians(-45)))
                .build();





        waitForStart();

        if(isStopRequested()) return;

        chassis.followTrajectory(trajectoryOneReposition);
        chassis.followTrajectory(lineUpBasketTrajectoryTwo);
        chassis.followTrajectory(lineupMiddleSampleTrajectoryThree);
        chassis.followTrajectory(goIntakeSampleTrajectoryFour);
        chassis.followTrajectory(lineupBasketTrajectoryFive);


       //     slides.setHeight(10);

//slides.setHeight(100);





            telemetry.addData("pose", chassis.getPoseEstimate());
            telemetry.update();
        }
    }