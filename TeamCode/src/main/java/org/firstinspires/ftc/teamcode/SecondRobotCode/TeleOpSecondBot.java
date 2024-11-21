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

    public void runOpMode() throws InterruptedException {

        motorLeftF = hardwareMap.get(DcMotor.class ,"motorLeftF");
        motorRightF = hardwareMap.get(DcMotor.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotor.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotor.class, "motorLeftB");

        chassis = new ChassisSubsystem(motorLeftF, motorRightF, motorLeftB,motorRightB);
        arm = new ArmSubsytem(armMotor);
        intake = new IntakeSubsystem (intakeMotor, liftMotor);

        waitForStart();
        double speedPwr;
        double strafePwr;
        double turnPwr;
        double armPwr;

        while (opModeIsActive()) {
            speedPwr = -gamepad1.left_stick_y;
            strafePwr = -gamepad1.left_stick_x;
            turnPwr = -gamepad1.right_stick_x;
            chassis.moveMechChassis(speedPwr, strafePwr, turnPwr);
            armPwr = -gamepad2.left_stick_y;
            //arm.
        }

        telemetry.addData("Status", "wobot is on :3");
        telemetry.update();
    }




}
