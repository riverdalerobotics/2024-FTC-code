package org.firstinspires.ftc.teamcode.RookieBotCode;

import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HelperFunctions;
import org.firstinspires.ftc.teamcode.SecondRobotCode.Constants;

public class ArmSubsystem {
    DcMotor armMotor;

    public ArmSubsystem(DcMotor armMotor){
        this.armMotor = armMotor;
    }

    //getPosition methods
    public double getPosInDegrees() {
        return armMotor.getCurrentPosition() * 360 / Constants.ArmConstants.GEAR_RATIO;
    }
    public double getPositionInTicks(){
        return armMotor.getCurrentPosition();
    }

    //unit conversions used in setting angle
    public double degToTicks(double deg){
        return deg/360*Constants.ArmConstants.GEAR_RATIO;
    }

    public void setArmAngle(double angleInDegrees, double speed){
        armMotor.setPower(speed);
        armMotor.setTargetPosition((int)degToTicks(angleInDegrees));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoder(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void emergencyStop() {
        armMotor.setPower(0);
    }
}




