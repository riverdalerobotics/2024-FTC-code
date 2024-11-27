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

    //TODO: code specific: configure imu, work on field centric code, copy and paste SampleMecanumDrive into ChassisSubsystem, create an auto, tune pidf for rr.
    //TODO: priotiy list: field centric, intake, arm, slides, road runner, buy coffee
    public void runOpMode() throws InterruptedException {

        motorLeftF = hardwareMap.get(DcMotorEx.class ,"motorLeftF");
        motorRightF = hardwareMap.get(DcMotorEx.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotorEx.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotorEx.class, "motorLeftB");
        armMotor =hardwareMap.get(DcMotor.class, "armMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
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
        double speedPwr;
        double strafePwr;
        double turnPwr;
        arm.resetEncoders();
        slides.resetEncoder();
        boolean armMove = false;

        //TODO: Figure out the controls
        while (opModeIsActive()) {

            //chassis
            speedPwr = -gamepad1.left_stick_y*0.2;
            strafePwr = -gamepad1.left_stick_x*0.2;
            turnPwr = -gamepad1.right_stick_x*0.2;
           // chassis.moveMechChassis(speedPwr, strafePwr, turnPwr);
            chassis.fieldOriented(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),speedPwr, strafePwr, turnPwr);
            if (gamepad1.start){
                imu.resetYaw();
            }
            if (gamepad1.x){
                slides.setHeight(500);
            }
            //arm
            if(slides.getCurrentHeight()>300){

                if (gamepad1.a){
                    intake.setWristPosition(0.8);}
                if(gamepad1.x){
                    intake.setWristPosition(0.2);

                }
                //intake
                if(gamepad1.b) {
                    intake.spinTake(1);
                }
                else if(gamepad1.y){
                    intake.spinTake(-1);
                } else{
                    intake.spinTake(0);
                }

                if (gamepad1.right_bumper){
                    arm.setArmAngle(127.76);
                    telemetry.addData("arm Pos", arm.getPosInDegrees());
                }
                else if(gamepad1.left_bumper){
                    arm.setArmAngle(90);

                }
            //slides

            //TODO: get wrist position, add the height of the slides to the height we want to go,

        }

            //wrist
            // telemetry.addData("yaw", imu.getRobotYawPitchRollAngles());
            telemetry.addData("Status", "wobot is on :3");
            telemetry.addData("current slide height mm", slides.getCurrentHeight());
            telemetry.addData("wrist current pos", intake.getWristPosition());
            telemetry.update();

        }
}}
