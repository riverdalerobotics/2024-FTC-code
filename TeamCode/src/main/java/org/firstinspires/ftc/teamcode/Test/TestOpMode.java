package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotCode.TeleopMode;
@TeleOp(name = "Test OpMode", group = "Linear OpMode")
public class TestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor testMotor;
        testMotor = hardwareMap.get(DcMotor.class, "TestMotor");
        waitForStart();
        while(opModeIsActive()){
            double power = gamepad1.left_stick_y;
            testMotor.setPower(power*0.1);
            telemetry.addData("Motor Position", testMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
