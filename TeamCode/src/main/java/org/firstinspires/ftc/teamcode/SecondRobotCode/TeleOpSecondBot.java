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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Second bot TeleOp", group="Linear OpMode")
public class TeleOpSecondBot extends  LinearOpMode {

    public DcMotorEx motorLeftF;
    public DcMotorEx motorRightF;
    public DcMotorEx motorRightB;
    public DcMotorEx motorLeftB;
    IMU imu;

    boolean fieldOriented = true;

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

    //TODO: code specific:  tune pidf for rr.
    //TODO: priotiy list:  road runner, buy coffee
    public void runOpMode() throws InterruptedException {

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

        imu = hardwareMap.get(IMU.class,"imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        chassis = new ChassisSubsystem(motorLeftF, motorRightF, motorLeftB,motorRightB,imu);
        arm = new ArmSubsystem(armMotor);
        intake = new IntakeSubsystem (intakeServo, wristServo);
        slides = new SlidesSubsystem(slideMotor, bucketServo);

        waitForStart();
        double speedPwr;
        double strafePwr;
        double turnPwr;
        arm.resetEncoders();
        slides.resetEncoder();

        //TODO: Figure out the controls
        while (opModeIsActive()) {

            //reset the yaw value
            if (gamepad1.start) {
                imu.resetYaw();
            }
            speedPwr = gamepad1.left_stick_y * 0.5;
            strafePwr = gamepad1.left_stick_x * 0.5;
            turnPwr = gamepad1.right_stick_x * 0.5;


            if (gamepad1.left_stick_button) {
                fieldOriented = true;
            } else if (gamepad1.right_stick_button) {
                fieldOriented = false;
            }
            if (fieldOriented) {
                chassis.fieldOriented(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), speedPwr, strafePwr, turnPwr);
            } else {
                chassis.moveMechChassis(speedPwr, strafePwr, turnPwr);
            }

            //intake and outtake
            if (gamepad1.left_bumper) {
                intake.spinTake(Constants.IntakeConstants.OUTAKE_SPEED);
            } else if (gamepad1.right_bumper) {
                intake.spinTake(Constants.IntakeConstants.INTAKE_SPEED);
            } else {
                intake.spinTake(0);
            }

            //intake
            while (gamepad1.x) {
                if (slides.getCurrentHeight() <= 230) {
                    slides.setHeight(400);
                } else if (slides.getCurrentHeight() > 230) {

                    arm.setArmAngle(Constants.ArmConstants.ARM_ANGLE_INTAKE);
                    intake.setWristPosition(Constants.IntakeConstants.WRIST_INTAKE_POSITION);

                }
            }

        //basket testing
        while(gamepad1.a){
            slides.setHeight(Constants.SlidesConstants.HIGH_BASKET_POSITION);
        }
        if(gamepad1.b){
            bucketServo.setPosition(Constants.BucketConstants.BUCKET_SCORE_POSITION);
        }

        //Sets the slides to a handoff position (receives the samples)
           while(gamepad1.y) {
               slides.setHeight(Constants.SlidesConstants.HANDOFF_POSITION);
               if(slides.getCurrentHeight()<230) {
                   arm.setArmAngle(Constants.ArmConstants.ARM_ANGLE_HANDOFF);

                   intake.setWristPosition(Constants.IntakeConstants.WRIST_HANDOFF_POSITION);
                   slides.bucketServo.setPosition(Constants.BucketConstants.BUCKET_HANDOFF_POSITION);
                  // intake.spinTake(-1);
               }

           }
           if(gamepad1.dpad_right){
               arm.emergencyStop();
           }

            // telemetry.addData("yaw", imu.getRobotYawPitchRollAngles());
            telemetry.addData("Status", "wobot is on :3");
            telemetry.addData("current arm angle", arm.getPosInDegrees());
            telemetry.addData("current slide height mm", slides.getCurrentHeight());
            telemetry.addData("wrist current pos", intake.getWristPosition());
            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles());
            telemetry.addData("robot oriented", fieldOriented);
            telemetry.update();



            }
    }


        }

