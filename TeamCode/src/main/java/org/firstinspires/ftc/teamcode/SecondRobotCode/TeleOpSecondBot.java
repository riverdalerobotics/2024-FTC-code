package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

@TeleOp(name="Second bot TeleOp", group="Linear OpMode")
public class TeleOpSecondBot extends  LinearOpMode {

    //chassis motors
    public DcMotorEx motorLeftF;
    public DcMotorEx motorRightF;
    public DcMotorEx motorRightB;
    public DcMotorEx motorLeftB;

    //arm motor
    private DcMotor armMotor;

    // slide motor
    private DcMotor armExtend;

    // intake and wrist servo
    private Servo liftServo;
    private CRServo intakeServo;

    public WebcamName camera;
    IMU imu;
    //BNO055IMU imu;
    VoltageSensor batteryVoltageSensor;

    //declare subsystems
    ChassisSubsystem chassis;
    ArmSubsystem arm;
    IntakeSubsystem intake;

    SlidesSubsystem slides;

    //TODO: code specific: configure imu, work on field centric code, copy and paste SampleMecanumDrive into ChassisSubsystem, create an auto, tune pidf for rr.
    //TODO: priotiy list: field centric, intake, arm, slides, road runner, buy coffee


    public void runOpMode() throws InterruptedException {

        //initialize your motors
        motorLeftF = hardwareMap.get(DcMotorEx.class, "motorLeftF");
        motorRightF = hardwareMap.get(DcMotorEx.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotorEx.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotorEx.class, "motorLeftB");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        intakeServo = hardwareMap.get(CRServo.class, "intake");
        // armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TODO: change the IMU from BNO055IMU to BHI260AP (has a quicker reaction time), and make sure we have the BHI260AP IMU
        //TODO: i'm pretty sure the BHI260AP IMU is just the universal one
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit= BNO055IMU.AngleUnit.DEGREES;
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        imu.initialize(parameters);

        //imu initialization
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        //voltage sensor initialization
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //initialize your subsystems
        chassis = new ChassisSubsystem(motorLeftF, motorRightF, motorLeftB, motorRightB, imu);
        arm = new ArmSubsystem(armMotor);
        intake = new IntakeSubsystem(intakeServo, liftServo);
        slides = new SlidesSubsystem(armExtend);


        waitForStart();
        double speedPwr;
        double strafePwr;
        double turnPwr;
        double slidePwrTemp;
        arm.resetEncoders();

        //TODO: Figure out the controls, most likely to be:
        /**
         * A for Intake
         * B for Spit
         * D pad Up to lift Arm to max Pos
         * D pad Down to set Arm to min Pos (basically laying on top of chassis)
         * Y for Lifting slides???
         *
         * Maybe we should use trigger buttons??
         * arm positons will most liekly be made autonmoous
         * //TODO: do we need a built in emergence STOP?
         *
         */


        while (opModeIsActive()) {

            /**
             * Chassis Logic
             * */
            speedPwr = -gamepad1.left_stick_y * 0.3;
            strafePwr = -gamepad1.left_stick_x * 0.3;
            turnPwr = -gamepad1.right_stick_x * 0.3;
            chassis.moveMechChassis(speedPwr, strafePwr, turnPwr);

            // chassis.fieldOriented(0,speedPwr, strafePwr, turnPwr);

            // slidePwrTemp = -gamepad2.right_stick_y;
            telemetry.addData("Xaxis SPeed", strafePwr);
            //telemetry.addData("ticks", motorLeftB.getCurrentPosition());
            telemetry.addData("Y axis Speed", speedPwr);

            /**
             * Arm Logic
             * */
            if (gamepad2.x) {
                arm.setArmAngle(Constants.ArmConstants.ARM_ANGLE_TEST);
            }
            if (gamepad2.y) {
                arm.setArmAngle(Constants.ArmConstants.ARM_ANGLE_INTAKE);
            }
//            if (gamepad2.a){
//                arm.setArmAngle(Constants.ArmConstants.ARM_ANGLE_HANDOFF);
//            }


            /**
             * lift servo
             * */
            //intake servo is spinning lift
            //THIS IS ACTUALLY INTAKE
            //TODO: currently set to lift servo but it spins intake??
//            if (gamepad1.a){
//                liftServo.setPosition(-0.3);}
//
//            }
//            else{
//                liftServo.setPosition(0);


            /**
             * Intake Logic
             * */
        }
        //lift servo is spinning intake
        //THIS IS ACTUALLY LIFT
//            if(gamepad1.b) {
//                intakeServo.setPower(0.5);
//            }
//            else if(gamepad1.y){
//                intakeServo.setPower(-0.5);
//            } else{
//                    intakeServo.setPower(0);
//
//            }


        telemetry.addData("Status","wobot is on :3");
        telemetry.addData("Current Arm Position:",arm.getPosInDegrees());

        telemetry.update();
    //telemetry.addData("Y axis Speed", speedPwr);
    }
}


