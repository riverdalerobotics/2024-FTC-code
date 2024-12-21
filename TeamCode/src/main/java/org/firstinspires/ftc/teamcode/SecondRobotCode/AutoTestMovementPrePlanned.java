package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


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
@Autonomous(group = "asdfasdfasdf")
public class AutoTestMovementPrePlanned extends LinearOpMode {

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

    @Override
    public void runOpMode() throws InterruptedException {

        //8.65 inches length from middle
        //8.9 inches wide from middle

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        motorLeftF = hardwareMap.get(DcMotorEx.class, "motorLeftF");
        motorRightF = hardwareMap.get(DcMotorEx.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotorEx.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotorEx.class, "motorLeftB");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        bucketServo = hardwareMap.get(Servo.class, "bucket");
        wristServo = hardwareMap.get(Servo.class, "wrist");
        intakeServo = hardwareMap.get(CRServo.class, "intake");


        chassis = new ChassisSubsystem(motorLeftF, motorRightF, motorLeftB, motorRightB, imu, batteryVoltageSensor);
        arm = new ArmSubsystem(armMotor);
        intake = new IntakeSubsystem(intakeServo, wristServo);
        slides = new SlidesSubsystem(slideMotor, bucketServo);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //starting pose, NOT RELATIVE TO FIELD, RELATIVE TO BOT'S STARTING POINT
        startingPose = new Pose2d(0, 0, Math.toRadians(0));

        chassis.setPoseEstimate(startingPose);




        //all trajectories used


        TrajectorySequence forwardOnce = chassis.trajectorySequenceBuilder(startingPose)
                .forward(4)

//                .addTemporalMarker(() -> slides.setHeight(Constants.SlidesConstants.HIGH_BASKET_POSITION))
//                .waitSeconds(10)
                  .addTemporalMarker(() -> arm.setArmAngle(Constants.ArmConstants.ARM_ANGLE_HANDOFF))

                .build();

        TrajectorySequence lineUpBucketTrajectoryOne = chassis.trajectorySequenceBuilder(forwardOnce.end())
                .lineToLinearHeading(
                        new Pose2d(6.54, 15, Math.toRadians(315)),
                        chassis.getVelocityConstraint(40, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(40)

                )

                .build();

        TrajectorySequence driveForwardToBasketScoreTrajectoryTwo = chassis.trajectorySequenceBuilder(lineUpBucketTrajectoryOne.end())
                .lineToLinearHeading(
                        new Pose2d(3, 18, Math.toRadians(315)),
                        chassis.getVelocityConstraint(10, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(50)
                )
                .waitSeconds(1) //TODO: probably remove

                .waitSeconds(2)
                .build();

        TrajectorySequence preIntakeTrajectoryThree = chassis.trajectorySequenceBuilder(driveForwardToBasketScoreTrajectoryTwo.end())
                .lineToSplineHeading(
                        new Pose2d(7,21, 0),
                        chassis.getVelocityConstraint(20, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(40)
                )

                .waitSeconds(2)
                .build();


        TrajectorySequence goIntakeMidSampleTrajectoryFour = chassis.trajectorySequenceBuilder(preIntakeTrajectoryThree.end())
                .lineToLinearHeading(
                        new Pose2d(22.2, 21, Math.toRadians(0)),
                        chassis.getVelocityConstraint(30, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(50)
                )
                .waitSeconds(0.5)

                .build();







// TODO: HOW DO WE INTKAE?
        TrajectorySequence lineUpBucketTrajectoryFive = chassis.trajectorySequenceBuilder(goIntakeMidSampleTrajectoryFour.end())
                .lineToLinearHeading(
                        new Pose2d(6.54, 15, Math.toRadians(315)),
                        chassis.getVelocityConstraint(40, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(50)
                )

                .build();

        TrajectorySequence driveForwardToBasketScoreTrajectorySix = chassis.trajectorySequenceBuilder(lineUpBucketTrajectoryFive.end())
                .lineToLinearHeading(
                        new Pose2d(3.7, 18, Math.toRadians(315)),
                        chassis.getVelocityConstraint(40, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(50)
                )

                .waitSeconds(1) //TODO: probably remove
                .build();






        TrajectorySequence preIntakeTrajectorySeven = chassis.trajectorySequenceBuilder(driveForwardToBasketScoreTrajectorySix.end())
                .lineToSplineHeading(
                        new Pose2d(7,12, 0),
                        chassis.getVelocityConstraint(20, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(40)
                )
                .waitSeconds(2)
                .build();



        TrajectorySequence goIntakeMidSampleTrajectoryEight = chassis.trajectorySequenceBuilder(preIntakeTrajectorySeven.end())
                .lineToLinearHeading(
                        new Pose2d(22.2, 10, Math.toRadians(0)),
                        chassis.getVelocityConstraint(30, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(50)
                )
                .waitSeconds(0.5)

                .build();


        TrajectorySequence lineUpBucketTrajectoryNine = chassis.trajectorySequenceBuilder(goIntakeMidSampleTrajectoryEight.end())
                .lineToLinearHeading(
                        new Pose2d(6.54, 15, Math.toRadians(315)),
                        chassis.getVelocityConstraint(40, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(40)

                )

                .build();

        TrajectorySequence driveForwardToBasketScoreTrajectoryTen = chassis.trajectorySequenceBuilder(lineUpBucketTrajectoryNine.end())
                .lineToLinearHeading(
                        new Pose2d(3, 18, Math.toRadians(315)),
                        chassis.getVelocityConstraint(10, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(50)
                )
                .waitSeconds(1) //TODO: probably remove

                .waitSeconds(2)
                .build();

        TrajectorySequence eleven = chassis.trajectorySequenceBuilder(driveForwardToBasketScoreTrajectoryTen.end())
                .lineToLinearHeading(
                        new Pose2d(14, 11.8, Math.toRadians(31)),
                        chassis.getVelocityConstraint(10, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(50)
                )
                .waitSeconds(1) //TODO: probably remove

                .waitSeconds(2)
                .build();

        TrajectorySequence twelve = chassis.trajectorySequenceBuilder(eleven.end())
                .lineToLinearHeading(
                        new Pose2d(20.5, 15.8, Math.toRadians(31)),
                        chassis.getVelocityConstraint(10, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(50)
                )
                .waitSeconds(1) //TODO: probably remove

                .waitSeconds(2)
                .build();


        TrajectorySequence lineUpBucketTrajectoryThirteen = chassis.trajectorySequenceBuilder(twelve.end())
                .lineToLinearHeading(
                        new Pose2d(6.54, 15, Math.toRadians(315)),
                        chassis.getVelocityConstraint(40, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(40)

                )

                .build();

        TrajectorySequence driveForwardToBasketScoreTrajectoryFourteen = chassis.trajectorySequenceBuilder(lineUpBucketTrajectoryThirteen.end())
                .lineToLinearHeading(
                        new Pose2d(3, 18, Math.toRadians(315)),
                        chassis.getVelocityConstraint(10, Math.toRadians(200), Constants.ChassisConstants.TRACK_WIDTH),
                        chassis.getAccelerationConstraint(50)
                )
                .waitSeconds(1) //TODO: probably remove

                .waitSeconds(2)
                .build();

        waitForStart();

//        while (opModeIsActive()) {
//            do {
//                intake.spinTake(Constants.IntakeConstants.INTAKE_SPEED);
//            } while(chassis.getPoseEstimate().getY()>20);
//
//            telemetry.addData("pose", chassis.getPoseEstimate());
//            telemetry.update();
//
//        }

        if (isStopRequested()) return;

        chassis.followTrajectorySequence(forwardOnce);
        chassis.followTrajectorySequence(lineUpBucketTrajectoryOne);
        chassis.followTrajectorySequence(driveForwardToBasketScoreTrajectoryTwo);
        chassis.followTrajectorySequence(preIntakeTrajectoryThree);
        chassis.followTrajectorySequence(goIntakeMidSampleTrajectoryFour);
        chassis.followTrajectorySequence(lineUpBucketTrajectoryFive);
        chassis.followTrajectorySequence(driveForwardToBasketScoreTrajectorySix);
        chassis.followTrajectorySequence(preIntakeTrajectorySeven);
        chassis.followTrajectorySequence(goIntakeMidSampleTrajectoryEight);
        chassis.followTrajectorySequence(lineUpBucketTrajectoryNine);
        chassis.followTrajectorySequence(driveForwardToBasketScoreTrajectoryTen);
        chassis.followTrajectorySequence(eleven);
        chassis.followTrajectorySequence(twelve);
        chassis.followTrajectorySequence(lineUpBucketTrajectoryThirteen);
        chassis.followTrajectorySequence(driveForwardToBasketScoreTrajectoryFourteen);

////
//        chassis.followTrajectorySequence(preIntakeRightTrajectorySeven);

    }
}
