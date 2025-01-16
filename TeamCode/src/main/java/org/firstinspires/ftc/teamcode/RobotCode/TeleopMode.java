package org.firstinspires.ftc.teamcode.RobotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotCode.ArmSubsystem;

@TeleOp (name = "Nicolas Teleop", group = "Linear OpMode")

public class TeleopMode extends LinearOpMode {

    boolean autoWheels = false;

    public double degToRotation(double deg) {
        return deg / 360 * 1425.2 * 5;
    }

    public double chassisDistance(double distance) {
        return distance / 537.7;
    }

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        waitForStart();
        DcMotor left;
        DcMotor right;
        DcMotor armPivot;
        Servo wrist;
        CRServo intakeServo;
        left = hardwareMap.get(DcMotor.class, "LeftMotor");
        right = hardwareMap.get(DcMotor.class, "RightMotor");
        armPivot = hardwareMap.get(DcMotor.class, "ArmPivot");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");
        ChassisSubsystem chassis = new ChassisSubsystem(left, right);
        ArmSubsystem arm = new ArmSubsystem(armPivot);
        Commands commands = new Commands(armPivot, wrist, intakeServo);
        OI oi = new OI(gamepad1, gamepad2);
        double armAngle = armPivot.getCurrentPosition() * 360 / (1425.2 * 5);
        armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boolean moveServo = false;
        boolean reverseDrive = false;
        wrist.setPosition(0.1);
        armPivot.setPower(0.8);
        armPivot.setTargetPosition((int) degToRotation(10));

        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // IntakeSubsystem intake = new IntakeSubsystem(intakeServo, wrist);
        while (opModeIsActive()) {
            // IntakeSubsystem.intakePowerOn(0);
//
//
//
//            }

            double speed = gamepad1.left_stick_y*0.7;
            double turn = gamepad1.right_stick_x*0.7;
            if (reverseDrive) {
                speed = -speed;
            }
            if (gamepad1.start) {
                reverseDrive = true;
            }
            if (gamepad1.back) {
                reverseDrive = false;
            }
            chassis.drive(speed, turn);


            //Score Low Bucket and High Bar angle
            if (gamepad2.b) {
                armPivot.setPower(0.8);
                armPivot.setTargetPosition((int) degToRotation(159.0));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //Intake Position
            if (gamepad2.a) {
                armPivot.setPower(0.8);
                armPivot.setTargetPosition((int) degToRotation(253.0));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(0.1);

            }

            //Climb
            if (gamepad1.dpad_down) {


                while (armPivot.isBusy()) {
                    armPivot.setPower(0.8);
                    armPivot.setTargetPosition((int) degToRotation(200));
                    armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                armPivot.setPower(0.8);
                armPivot.setTargetPosition((int) degToRotation(0));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }



            //outtake
            if (gamepad2.x) {
                intakeServo.setPower(0.8);
                //intake
            } else if (gamepad2.y) {
                intakeServo.setPower(-0.8);
            }

            //score Bar angle
            else if (gamepad2.dpad_right) {
                armPivot.setPower(0.6);
                armPivot.setTargetPosition((int) degToRotation(183));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeServo.setPower(-0.8);
                //score bucket front position
            } else if (gamepad2.dpad_left) {
                armPivot.setPower(0.8);
                armPivot.setTargetPosition((int) degToRotation(100));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(-0.4);
                intakeServo.setPower(-8);
            } else {
                intakeServo.setPower(0);
            }


            //move Wrist
            if (gamepad2.left_bumper) {
                wrist.setPosition(0.45);
            }
            //reset Wrist
            if (gamepad2.right_bumper) {
                wrist.setPosition(0.1);
            }
            // puts arm back down
            if (gamepad2.dpad_up) {
                armPivot.setPower(0.8);
                armPivot.setTargetPosition((int) degToRotation(10));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.dpad_down){
                armPivot.setPower(0.8);
                armPivot.setTargetPosition((int) degToRotation(240));
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }



            telemetry.addData("Motor Position", armPivot.getCurrentPosition() * 360 / (1425.2 * 5));
            telemetry.addData("WRIST POS", wrist.getPosition());
            telemetry.addData("left wheel value", left.getCurrentPosition());
            telemetry.update();
        }


    }
}






