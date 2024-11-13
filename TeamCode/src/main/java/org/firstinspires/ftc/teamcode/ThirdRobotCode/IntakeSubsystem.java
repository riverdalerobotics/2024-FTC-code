package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    CRServo intakeServo;
    Servo wrist;
    ColorSensor colour;
    public IntakeSubsystem (CRServo intakeMotor, Servo wristServo, ColorSensor colorSensor){
        this.wrist = wristServo;
        this.intakeServo = intakeMotor;
        this.colour = colorSensor;
    }
    public void pivotIntake(double angle){
        if (angle>180){
            angle = angle-360;
        }
        double newAngle = angle/180;
        wrist.setPosition(newAngle);
    }
    public void spinIntake(double power){
        intakeServo.setPower(power);
    }
    public char getColour(){
        char color = 'N';
        double distToRed =(255-colour.red());
        double distToYellow = 255-colour.green();
        double distToBlue = 255-colour.blue();
        double closestColour = Math.min(distToYellow,Math.min(distToBlue, distToRed));
        if (closestColour == distToRed){
            color = 'r';
        }
        else if (closestColour == distToBlue){
            color = 'b';
        }

        else if (closestColour == distToYellow){
            color = 'y';
        }
        else{
            color = 'N';}
        return color;
    }
}
