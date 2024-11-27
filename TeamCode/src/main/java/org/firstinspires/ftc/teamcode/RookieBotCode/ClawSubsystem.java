package org.firstinspires.ftc.teamcode.RookieBotCode;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem {


    Servo clawServo;

    public ClawSubsystem(Servo claw){
        this.clawServo = claw;

    }
    public void setClawPosition(double Position){

        clawServo.setPosition(Position);
    }
}
