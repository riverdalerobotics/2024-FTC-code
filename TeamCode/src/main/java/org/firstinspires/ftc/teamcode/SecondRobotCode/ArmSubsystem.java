package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HelperFunctions;

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

    public void setArmAngle(double angleInDegrees){
    armMotor.setPower(0.7);
    armMotor.setTargetPosition((int)degToTicks(angleInDegrees));
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoders(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void emergencyStop() {
        armMotor.setPower(0);
    }
}


