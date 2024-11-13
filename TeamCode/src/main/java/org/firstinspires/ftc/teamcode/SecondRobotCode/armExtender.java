package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ArmExtender;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class armExtender {
    static DcMotor armExtenderMotor;

    armExtender(DcMotor armExtMotor){
        this.armExtenderMotor = armExtMotor;
    }


    public static void armExtends() {

        double encoderValue = armExtenderMotor.getCurrentPosition();
        double motorRotation = ArmExtender.WHEEDIAMITER;

        double measurement = motorRotation*encoderValue;

        armExtenderMotor.setTargetPosition((int) measurement);
        armExtenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Measurement moved", measurement);


        }

        public static void armUnextends() {
        double encoderValue = armExtenderMotor.getCurrentPosition();
        double motorRotation = ArmExtender.WHEEDIAMITER;
        double measurement = motorRotation*encoderValue;
        armExtenderMotor.setTargetPosition((int) measurement);
        armExtenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
}

