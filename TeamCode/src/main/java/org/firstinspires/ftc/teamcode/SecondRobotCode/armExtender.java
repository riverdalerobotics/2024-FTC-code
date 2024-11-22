package org.firstinspires.ftc.teamcode.SecondRobotCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.SecondRobotCode.Constants.ArmExtender;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


//TODO: WHO EVER SEES THIS NEXT CHANGE THE FILE PLEASE IT SHOULD BE 'ArmExtender' NOT 'armExtender' :)

public class armExtender {
    static DcMotor armExtendMotor;

    public armExtender(DcMotor armExtend){
        this.armExtendMotor = armExtend;
    }

    public static void armExtends() {

        double encoderValue = armExtendMotor.getCurrentPosition();
        double motorRotation = ArmExtender.WHEEDIAMITER;

        double measurement = motorRotation*encoderValue;

        armExtendMotor.setTargetPosition((int) measurement);
        armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Measurement moved", measurement);


        }

        public static void armExt(double power){
            armExtendMotor.setPower(power);


    }

        public static void armUnextends() {
        double encoderValue = armExtendMotor.getCurrentPosition();
        double motorRotation = ArmExtender.WHEEDIAMITER;
        double measurement = motorRotation*encoderValue;
        armExtendMotor.setTargetPosition((int) measurement);
        armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
}

