package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Second bot TeleOp", group="Linear OpMode")
public class TeleOpSecondBot extends  LinearOpMode {

    public DcMotorEx motorLeftF;
    public DcMotorEx motorRightF;
    public DcMotorEx motorRightB;
    public DcMotorEx motorLeftB;
    IMU imu;
    private VoltageSensor batteryVoltageSensor;

    boolean fieldOriented= true;

    private DcMotor armMotor;

    private DcMotor slideMotor;
    private Servo bucketServo;

    private Servo wristServo;
    private CRServo intakeServo;
    private double maxSpeed =1;

    public WebcamName camera;

    ChassisSubsystem chassis;
    ArmSubsystem arm;
    IntakeSubsystem intake;
    SlidesSubsystem slides;

    double speedPwr;
    double strafePwr;
    double turnPwr;

    public void runOpMode() throws InterruptedException {

        motorLeftF = hardwareMap.get(DcMotorEx.class, "motorLeftF");
        motorRightF = hardwareMap.get(DcMotorEx.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotorEx.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotorEx.class, "motorLeftB");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        bucketServo = hardwareMap.get(Servo.class, "bucket");
        wristServo = hardwareMap.get(Servo.class, "wrist");
        intakeServo = hardwareMap.get(CRServo.class, "intake");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        chassis = new ChassisSubsystem(motorLeftF, motorRightF, motorLeftB, motorRightB, imu, batteryVoltageSensor);
        arm = new ArmSubsystem(armMotor);
        intake = new IntakeSubsystem(intakeServo, wristServo);
        slides = new SlidesSubsystem(slideMotor, bucketServo);

        waitForStart();
        arm.resetEncoders();
        slides.resetEncoder();

        while (opModeIsActive()) {
            //reset the yaw value
            if (gamepad2.start) {
                imu.resetYaw();
            }
            if (gamepad2.left_bumper) {
                maxSpeed = 0.3;

            } else {
                maxSpeed = 0.8;
            }

            speedPwr = gamepad2.left_stick_y * maxSpeed;
            strafePwr = gamepad2.left_stick_x * maxSpeed;
            turnPwr = gamepad2.right_stick_x * maxSpeed;

            if (gamepad2.left_stick_button) {
                fieldOriented = true;
            } else if (gamepad2.right_stick_button) {
                fieldOriented = false;
            }

            if (fieldOriented) {
                chassis.fieldOriented(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), speedPwr, strafePwr, turnPwr);
            } else if (!fieldOriented) {
                chassis.moveMechChassis(speedPwr, strafePwr, turnPwr);
            }

            //BASKET TO SCORING ON DRIVING CONTROLLER

            //TODO: 260 IS THE HEIGHT SLIDES NEED TO BE FOR ARM TO PASS OFF (STORE AS A CONSTANT)
            if (gamepad2.a) {
                if (arm.getPosInDegrees() >= 88) {
                    arm.setArmAngle(96);
                    slides.setHeight(Constants.SlidesConstants.HIGH_BASKET_POSITION);
                }
            }

            // score the basket by clipping bucket servo
            if (gamepad2.b) {
                bucketServo.setPosition(Constants.BucketConstants.BUCKET_SCORE_POSITION);
            }

            //intake and outtake
            if (gamepad1.left_bumper) {
                intake.spinTake(Constants.IntakeConstants.OUTAKE_SPEED);
            } else if (gamepad1.right_bumper) {
                intake.spinTake(Constants.IntakeConstants.INTAKE_SPEED);
            } else {
                intake.spinTake(0);
            }

            //ONLY NEEDS TO BE RAN ONCE: Arm to Rest Position (ARM TO 90 DEGREES)
            if (gamepad1.left_stick_button) {
                if (slides.getCurrentHeight() <= 230) {
                    slides.setHeight(400);
                } else if (slides.getCurrentHeight() > 230) {
                    arm.setArmAngle(Constants.ArmConstants.ARM_ANGLE_SLIDE_GOING_UP);
                    intake.setWristPosition(Constants.IntakeConstants.WRIST_HANDOFF_POSITION);

                }
            }

            //Sets the slides to a handoff position (receivgames the samples) SLIDES TO 0 ARMS TO 70 DEGREES WRIST TO 0
            if (gamepad1.y) {
                if (arm.getPosInDegrees() >= 88) {
                    slides.setHeight(Constants.SlidesConstants.HANDOFF_POSITION);
                    slides.bucketServo.setPosition(Constants.BucketConstants.BUCKET_HANDOFF_POSITION);

                }

            }

            // ARM TO PRE INTAKE ENTER SUBMERSIBLE DEGREES AND WRIST TO 0.71
            if (gamepad1.a) {
                if (arm.getPosInDegrees() >67 ) {
                    arm.setArmAngle(193);
                    intake.setWristPosition(Constants.IntakeConstants.WRIST_INTAKE_POSITION);
                }
            }

            // ARM TO INTAKE ARM TO 211 DEGREES AND WRIST TO 0.71
            if (gamepad1.x) {
                if (arm.getPosInDegrees() > 70) {
                    arm.setArmAngle(Constants.ArmConstants.ARM_ANGLE_INTAKE);
                    intake.setWristPosition(Constants.IntakeConstants.WRIST_INTAKE_POSITION);
                }
            }

//            //TODO: TEST EMERGENCY CONTROLS
//        if (gamepad1.start) {
//            arm.emergencyStopBrandon(arm.getPosInDegrees());
//            slides.emergencyStopBrandon(slides.getCurrentHeight());
//        }
//
//        if (gamepad1.dpad_right){
//            arm.setArmAngle(arm.getPosInDegrees()+1);
//        }
//        if (gamepad1.dpad_left){
//            arm.setArmAngle(arm.getPosInDegrees()-1);
//        }
//        if (gamepad1.dpad_down){
//            slides.setHeight(slides.getCurrentHeight()-1);
//        }
//        if (gamepad1.dpad_up){
//            slides.setHeight(slides.getCurrentHeight()+1);
//        }

        // telemetry.addData("yaw", imu.getRobotYawPitchRollAngles());
        telemetry.addData("Status", "wobot is on :3");
        telemetry.addData("current arm angle", arm.getPosInDegrees());
        telemetry.addData("current slide height mm", slides.getCurrentHeight());
        telemetry.addData("wrist current pos", intake.getWristPosition());
        telemetry.addData("yaw", imu.getRobotYawPitchRollAngles());
        telemetry.addData("Field Oriented is enable?", fieldOriented);
        telemetry.update();
    }
        }
    }
