package org.firstinspires.ftc.teamcode.RookieBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="Rookie Bot TeleOP")
public class RookieTeleOp extends LinearOpMode {

    DcMotor leftDrive;
    DcMotor rightDrive  ;
    DcMotor arm;
    Servo claw;
    CRServo intake;
    ChassisSubsystem chassis;
    ArmSubsystem armSub;
    IntakeSubsystem intakeSub;
    DcMotor intakeMotor;

    public void runOpMode() throws InterruptedException {

        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        armSub = new ArmSubsystem(arm);
        intakeSub = new IntakeSubsystem(intake, claw, intakeMotor);
        chassis = new ChassisSubsystem(leftDrive, rightDrive);
        waitForStart();
        armSub.resetEncoder();
        armSub.setArmAngle(10,1);

        double speed;
        double turn;

        while(opModeIsActive()){

            //driver

            if(gamepad1.right_bumper){
                speed = gamepad1.left_stick_y/2;
            }
            else {
                speed = gamepad1.left_stick_y;
            }
            if(gamepad1.right_bumper){
                turn = gamepad1.right_stick_x/2;
            }
            turn = gamepad1.right_stick_x;

            if(gamepad2.a){
                armSub.setArmAngle(30, 1);
                intakeSub.moveIntakeArmToPos(140, 0.8);
            }
            if( gamepad2.right_bumper){
                intakeSub.spinTake(3.5);
            }
            else if(gamepad2.left_bumper){
                intakeSub.spinTake(-3.5);
            }
            else{
                intakeSub.spinTake(0);

            }

            //operator

            if(gamepad2.x) {
                intakeSub.moveIntakeArmToPos(0, 1);

                armSub.setArmAngle(80, 1);
            }

            if(gamepad2.dpad_up){
                intakeSub.grabWithClaw(0.35);
            }
            else{
                intakeSub.grabWithClaw(0.45);
            }
            if(gamepad2.y){
                armSub.setArmAngle(120, 1);
            }
            if(gamepad2.b){
                intakeSub.moveIntakeArmToPos(0,0.5);
                armSub.setArmAngle(10,0.5);
            }

            if(gamepad2.dpad_left){
                armSub.setArmAngle(52, 0.5);
            }

            if(gamepad2.left_trigger > 0.5){
                armSub.setArmAngle(170, 0.5);
                intakeSub.moveIntakeArmToPos(80, 0.5);
            }


            chassis.drive(speed,turn);
            telemetry.addData("Intake pos", intakeSub.intakeInDeg());
            telemetry.addData("Arm Position", armSub.getPosInDegrees());
            telemetry.addData("Y axis Pwr", speed);
            telemetry.addData("X axis Pwr", turn);
            telemetry.update();
        }
    }
}