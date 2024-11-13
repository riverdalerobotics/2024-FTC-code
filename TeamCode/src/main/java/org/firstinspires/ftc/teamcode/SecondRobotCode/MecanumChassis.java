package org.firstinspires.ftc.teamcode.SecondRobotCode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Sana & Jack Mecanum", group="Linear OpMode")
public class MecanumChassis extends LinearOpMode {

    public DcMotor motorLeftF;
    //Right Front motor is connected to port 0
    public DcMotor motorRightF;
    public DcMotor motorRightB;
    //Right Back Motor is connected to port 1
    public DcMotor motorLeftB;


    double leftFront;
    double leftBack;
    double rightFront;
    double rightBack;

    public void moveMecChassis() {
        double speed = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        leftFront = speed+turn +strafe;
        rightFront = speed-turn-strafe;
        leftBack = speed+turn-strafe;
        rightBack = speed-turn+strafe;

        motorLeftF.setPower(leftFront);
        motorLeftB.setPower(leftBack);
        motorRightF.setPower(rightFront);
        motorRightB.setPower(rightBack);

    }

    public void runOpMode() throws InterruptedException {
        motorLeftF = hardwareMap.get(DcMotor.class ,"motorLeftF");
        motorRightF = hardwareMap.get(DcMotor.class, "motorRightF");
        motorRightB = hardwareMap.get(DcMotor.class, "motorRightB");
        motorLeftB = hardwareMap.get(DcMotor.class, "motorLeftB");
        waitForStart();
        while (opModeIsActive()){

            telemetry.addData("Status", "wobot is on :3");
            moveMecChassis();
            telemetry.update();

        }

        }
    }

