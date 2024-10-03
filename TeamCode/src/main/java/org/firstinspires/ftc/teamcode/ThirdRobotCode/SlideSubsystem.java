package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import static org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants.ArmSubsystem.WHEEL_DIAMETER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.SecondRobotCode.Constants;

public class SlideSubsystem {
    DcMotor rightSlideExtend;
    DcMotor leftSlideExtend;

    public SlideSubsystem(DcMotor rightExtend, DcMotor leftExtend){
        this.leftSlideExtend = leftExtend;
        this.rightSlideExtend = rightExtend;

        this.leftSlideExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightSlideExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        // may reverse these
        // set initial position for motors
    }
    public void moveSlide(double power){
        leftSlideExtend.setPower(power);
        rightSlideExtend.setPower(power);
    }

    public void extendSlide(double motorRotation){
        motorRotation = leftSlideExtend.getCurrentPosition();

    }


}