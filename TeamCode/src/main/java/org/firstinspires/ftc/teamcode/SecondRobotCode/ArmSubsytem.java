package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmSubsytem {
    DcMotor armMotor;

    /**
     * @return the position of the arm in angles
     */
    public double armPos(){
     double pos = armMotor.getCurrentPosition()/(Constants.ArmSubsystem.GEARRATIO*360);
     return pos;
    }

    public void gotoPosition() {

    }


}
