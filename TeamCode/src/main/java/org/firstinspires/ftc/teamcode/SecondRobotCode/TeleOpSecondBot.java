package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="Second bot TeleOp", group="Linear OpMode")
public class TeleOpSecondBot extends  LinearOpMode {

    // What are the ports for the Chassis Motors??
    public DcMotor motorLeftF;
    public DcMotor motorRightF;
    public DcMotor motorRightB;
    public DcMotor motorLeftB;

    private DcMotor armMotor;

    // arm extender ??
    private DcMotor armExtend;

    private CRServo liftMotor;
    private CRServo intakeMotor;

    public WebcamName camera;


    ChassisSubsystem chassis;
    ArmSubsytem arm;
    IntakeSubsystem intake;

    //Angus if you see this we have to chane the file to "ArmExtender" because classes have a capitalized name
    armExtender armExtender;

    //TODO: code specific: configure imu, work on field centric code, copy and paste SampleMecanumDrive into ChassisSubsystem, create an auto, tune pidf for rr.
    //TODO: priotiy list: field centric, intake, arm, slides, road runner, buy coffee
    public void runOpMode() throws InterruptedException {

        motorLeftF = hardwareMap.get(DcMotor.class ,"motorLeftF");
        motorRightF = hardwareMap.get(DcMotor.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotor.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotor.class, "motorLeftB");
        armMotor =hardwareMap.get(DcMotor.class, "armMotor");

        chassis = new ChassisSubsystem(motorLeftF, motorRightF, motorLeftB,motorRightB);
        arm = new ArmSubsytem(armMotor);
        intake = new IntakeSubsystem (intakeMotor, liftMotor);

        waitForStart();
        double speedPwr;
        double strafePwr;
        double turnPwr;
        double armPwr;
        double intakePwr;
        double wristPwr;
        double slidePwrTemp;

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
            speedPwr = -gamepad1.left_stick_y*0.3;
            strafePwr = -gamepad1.left_stick_x*0.3;
            turnPwr = -gamepad1.right_stick_x*0.3;
            chassis.moveMechChassis(speedPwr, strafePwr, turnPwr);
            slidePwrTemp = -gamepad2.right_stick_y;
            intakePwr = 0.2;
            wristPwr = 0.3;

            armExtender.armExt(slidePwrTemp);
            telemetry.addData("Y axis Speed", speedPwr);
            telemetry.update();
            telemetry.addData("Arm Pos", arm.getPos());
            // telemetry.addData("Xaxis SPeed", strafePwr);
            //telemetry.addData("ticks", motorLeftB.getCurrentPosition());

            if(gamepad1.a){
                telemetry.addData("Arm Pos", arm.getPos());
                telemetry.update();
                arm.moveArmTest(30);
               // arm.gotoPos(30);
                telemetry.addData("Arm Pos", arm.getPos());
                telemetry.update();
            }
            if(gamepad1.b){
              //  telemetry.addData();
                intake.intakeSpin(intakePwr);
            }
            if(gamepad1.y){
                intake.Up(0.2);
            }


        }

        telemetry.addData("Status", "wobot is on :3");
        //telemetry.addData("Y axis Speed", speedPwr);
        telemetry.update();
    }




}
