package org.firstinspires.ftc.teamcode.SecondRobotCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SlidesSubsystem {
    static DcMotor slidesMotor;

    public SlidesSubsystem(DcMotor slide){
        this.slidesMotor = slide;
    }

    public static void armExtends() {

        double encoderValue = slidesMotor.getCurrentPosition();
        double motorRotation = Constants.ArmExtender.WHEEDIAMITER;

        double measurement = motorRotation*encoderValue;

        slidesMotor.setTargetPosition((int) measurement);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Measurement moved", measurement);


    }

    public static void armExt(double power){
        slidesMotor.setPower(power);


    }

    public static void armUnextends() {
        double encoderValue = slidesMotor.getCurrentPosition();
        double motorRotation = Constants.ArmExtender.WHEEDIAMITER;
        double measurement = motorRotation*encoderValue;
        slidesMotor.setTargetPosition((int) measurement);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
