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

    private Servo liftServo;
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
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        intakeServo = hardwareMap.get(CRServo.class, "intake");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        chassis = new ChassisSubsystem(motorLeftF, motorRightF, motorLeftB,motorRightB,imu);
        arm = new ArmSubsystem(armMotor);
        intake = new IntakeSubsystem (intakeServo, liftServo);
        slides = new SlidesSubsystem(slideMotor);

        waitForStart();
        double speedPwr;
        double strafePwr;
        double turnPwr;
        arm.resetEncoders();

        //TODO: Figure out the controls
        while (opModeIsActive()) {

            speedPwr = -gamepad1.left_stick_y*0.1;
            strafePwr = -gamepad1.left_stick_x*0.1;
            turnPwr = -gamepad1.right_stick_x*0.1;
           // chassis.moveMechChassis(speedPwr, strafePwr, turnPwr);
            chassis.fieldOriented(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),speedPwr, strafePwr, turnPwr);
            if (gamepad1.start){
                imu.resetYaw();
            }
            if (gamepad1.x){
                arm.setArmAngle(90);
            }
            if (gamepad1.a){
                intake.Up(0.5);}

            if(gamepad1.b) {
                intake.spinTake(0.5);
            }
            else if(gamepad1.y){
                intake.spinTake(-0.5);
            } else{
                intake.spinTake(0);
            }
            }
            telemetry.addData("Status", "wobot is on :3");
            telemetry.update();
    }
}
