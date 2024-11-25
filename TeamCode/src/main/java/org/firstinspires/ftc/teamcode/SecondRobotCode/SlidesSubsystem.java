package org.firstinspires.ftc.teamcode.SecondRobotCode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.analysis.function.Constant;

public class SlidesSubsystem {
    static DcMotor slidesMotor;

    public SlidesSubsystem(DcMotor slide) {
        this.slidesMotor = slide;
        slidesMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    //slides length:

    //unit conversions
    public double ticksToMillimeter(double ticks) {
        return ticks * (Constants.SlidesConstants.DISTANCE_PER_ROTATION/Constants.SlidesConstants.TICKS_PER_ROTATION);
    }
    public double millimeterToTicks(double mm) {
        return mm * (Constants.SlidesConstants.TICKS_PER_ROTATION / Constants.SlidesConstants.DISTANCE_PER_ROTATION);
    }


    public double getCurrentHeight(){
        return ticksToMillimeter(slidesMotor.getCurrentPosition());
    }

    public void setHeight(double heightMillimeter) {
        slidesMotor.setPower(0.3);
        slidesMotor.setTargetPosition((int) millimeterToTicks(heightMillimeter));
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//    public static void setSlidePower(double power){
//        slidesMotor.setPower(power);
//  }


    public void resetEncoder(){
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
