package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    CRServo intakeServo;
    Servo leftWrist;
    Servo rightWrist;
    RevColorSensorV3 colour;
    Telemetry telemetry;
    public IntakeSubsystem (CRServo intakeMotor, Servo leftWrist, RevColorSensorV3 colorSensor, Servo rightWrist, Telemetry telemetry){
        this.leftWrist = leftWrist;
        this.rightWrist = rightWrist;
        this.intakeServo = intakeMotor;
        this.colour = colorSensor;
        this.telemetry = telemetry;
        rightWrist.setDirection(Servo.Direction.REVERSE);
    }
    public void pivotIntake(double angle){
        leftWrist.setPosition(angle);
        rightWrist.setPosition(angle);
    }
    public void spinIntake(double power){
        intakeServo.setPower(power);
    }

    public char getColour(){
        char color = 'N';
        NormalizedRGBA normalizedcolours = colour.getNormalizedColors();
        double [] rgb = {colour.red(), colour.green(), colour.blue()};
        double distToWhite = Math.sqrt(Math.pow(400-rgb[0], 2)+Math.pow(400-rgb[1], 2)+Math.pow(400-rgb[2], 2));
        double distToRed = Math.sqrt(Math.pow(1000-rgb[0], 2)+Math.pow(500-rgb[1], 2)+Math.pow(500-rgb[2], 2));
        double distToYellow = Math.sqrt(Math.pow(1000-rgb[0], 2)+Math.pow(1000-rgb[1], 2)+Math.pow(500-rgb[2], 2));
        double distToBlue = Math.sqrt(Math.pow(500-rgb[0], 2)+Math.pow(500-rgb[1], 2)+Math.pow(1000-rgb[2], 2));
        double closestColour = Math.min(Math.min(distToYellow, distToWhite),Math.min(distToBlue, distToRed));
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
            color = 'w';}
        return color;
    }

    @Override
    public void periodic() {
        telemetry.addData("Color", getColour());
        double [] rgb = {colour.red(), colour.green(), colour.blue()};
        telemetry.addData("R:", rgb[0]+" G " +rgb[1]+ " B " + rgb[2]);

    }
}
