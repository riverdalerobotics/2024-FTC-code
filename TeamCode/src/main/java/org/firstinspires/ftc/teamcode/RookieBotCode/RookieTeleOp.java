package org.firstinspires.ftc.teamcode.RookieBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name="Rookie Bot TeleOP")
public class RookieTeleOp extends LinearOpMode {

        DcMotor left;
        DcMotor right;
        ChassisSubsystem chassis;

    public void runOpMode() throws InterruptedException {


        left = hardwareMap.get(DcMotor.class, "LeftMotor");
        right = hardwareMap.get(DcMotor.class, "RightMotor");
        waitForStart();

        double speed;
        double turn;

        while(opModeIsActive()){
            speed = -gamepad1.left_stick_y*0.3;
            turn = gamepad1.right_stick_x*0.3;

            chassis.drive(speed,turn);
            telemetry.addData("Y axis Pwr", speed);
            telemetry.addData("X axis ", turn);
            telemetry.update();
        }

    }
}
