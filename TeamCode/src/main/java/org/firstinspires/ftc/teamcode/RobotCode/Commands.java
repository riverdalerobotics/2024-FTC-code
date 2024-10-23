package org.firstinspires.ftc.teamcode.RobotCode;
import org.firstinspires.ftc.teamcode.RobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.RobotCode.IntakeSubsystem;
public class Commands {
    public static void scoreBucket(){
        //code to score bucket
        //TODO: Instead of 90 make the desired angle a constant in Constants.java-Nicolas
        ArmSubsystem.moveArm(90);
    }
}
