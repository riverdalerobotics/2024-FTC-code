package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ArmExtender;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class armExtender {
    DcMotor armExtenderMotorRight;
    DcMotor getArmExtenderMotorLeft;

    public void armExtends() {

        double encoderValue = armExtenderMotorRight.getCurrentPosition();
        double motorRotation = ArmExtender.WHEEDIAMITER;

        double measurement = motorRotation*encoderValue;

        armExtenderMotorRight.setTargetPosition( );


        telemetry.addData("Measurement moved", measurement);


        }
}

