package org.firstinspires.ftc.teamcode.SecondRobotCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SlidesSubsystem {
    static DcMotor slidesMotor;

    public SlidesSubsystem(DcMotor slide){
        this.slidesMotor = slide;
    }

    public static void setSlideHeight() {

        double encoderValue = slidesMotor.getCurrentPosition();
        double motorRotation = Constants.SlidesConstants.WHEEL_DIAMETER;

        double measurement = motorRotation*encoderValue;

        slidesMotor.setTargetPosition((int) measurement);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Measurement moved", measurement);


    }

    public static void setSlidePower(double power){
        slidesMotor.setPower(power);
    }

}
