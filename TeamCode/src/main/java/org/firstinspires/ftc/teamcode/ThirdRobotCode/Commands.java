package org.firstinspires.ftc.teamcode.ThirdRobotCode;


import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Commands {
    public static void climbPtOne(){
        SlideSubsystem.goToPos(Constants.SlideConstants.CLIMB_UP);
    }
    public void climbPtTwo(IMU gyro){
        ArmSubsystem.pivotArm(Constants.ArmConstants.CLIMB_UP_ANGLE, Constants.ArmConstants.CLIMB_SPEED);
        double pitch = ChassisSubsystem.pitch(gyro);

        double startPitch = pitch;
        while(pitch<45+startPitch){
           pitch = ChassisSubsystem.pitch(gyro);
        }
        SlideSubsystem.goToPos(Constants.SlideConstants.CLIMB_DOWN);
        while(ArmSubsystem.getPos()<Constants.ArmConstants.START_CLIMB){
        }
        ArmSubsystem.pivotArm(Constants.ArmConstants.CLIMB_DOWN_ANGLE, Constants.ArmConstants.CLIMB_SPEED);
    }

}
