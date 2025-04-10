package org.firstinspires.ftc.teamcode.ThirdRobotCode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Constants {
    /**
     * All constants needed for the chassis
     * */
    @Config
    static class ChassisConstants{
        public static   int WHEELDIAMATER = 0; //TODO: find this number
        public static   double WHEEL_CIRCUMFERENCE = WHEELDIAMATER* Math.PI;
        public static   double CHASSIS_WIDTH = 0; //TODO: find this number
        public static   double CHASSIS_LENGTH = 0; // TODO: find this number
        public static   double DRIVE_KP = 0.3;
        public static   double ROTATION_KP = 0.35;
        public static   double[] SCORE_POS = {-60, -35, 180};



    }
    /**
     * All constants needed for the the arm
     * */
    @Config
    public static class ArmConstants{
        public static PIDFCoefficients armPID = new PIDFCoefficients(3.75, 1.75, 1.4, 25);
        public static double START_CLIMB = 0; // TODO: find this number
        public static   double  GEAR_RATIO = (81/7)*537.7; //TODO: get this number
        public static   double CLAW_SERVO_START_POSITION = 0; //TODO: get this number
        public static   double WHEEL_DIAMETER = 0; //TODO: get this number
        public static   double CLIMB_UP_ANGLE = 100; //TODO: find this number
        public static   double CLIMB_DOWN_ANGLE = 0d; //TODO: find this number
        public static   double REST_SPEED = 1; //TODO: find this number
        public static   double SCORE_SPEED = 0;//TODO: find this number
        public static   double BUCKET_ANGLE = 95;//TODO: find this number
        public static   double INTAKE_ANGLE = 0;//TODO: find this number
        public static   double INTAKE_SPEED = 0;//TODO: find this number

    }
    @Config
    public static class SlideConstants {
        public static   double ZERO = 10;
        public static   double INTAKE_MIN = 0;//TODO: find this number
        public static   double INTAKE_MAX = 60;//TODO: find this number
        public static   double TOLERANCE = 4;//TODO: find this number
        public static   double kp = 0.1;//TODO: find this number
        public static   double ki = 0;//TODO: find this number
        public static   double kd = 0;//TODO: find this number
        public static   double kf = 0;//TODO: find this number
        public static   PIDFCoefficients slidesPID = new PIDFCoefficients(kp, ki, kd, kf);
        public static   int SPEED = 0;//TODO: find this number
        public static   double ABSOLUTE_LIMIT = 0;//TODO: find this number
        public static   int FORWARD_LIMIT = 0; //TODO: get this number
        public static   int BACKWARD_LIMIT = 0; // TODO: get this number
        public static   double GEARDIAMETER = (3.8*Math.PI)/384.5; //TODO: get this number
        public static   double LIMIT = 41-Math.cos(85*Math.PI/180);
        public static   double ARM_LENGTH = 0; //TODO: get this number
        public static   double CLIMB_UP = 40; //TODO: find this number
        public static   double CLIMB_DOWN = 10;//TODO: find this number
        public static   double SCORE_BUCKET = 76;//TODO: find this number
        public static   double INTAKE_POSITION = 0;//TODO: find this number
        public static   double INTAKE_SPEED = -0.3; //TODO: find this number
    }
    @Config
    public static class IntakeConstants{
        public static double max = 0.2;
        public static double min = 0.5;
        public static   int WAIT_TIME = 200;//TODO: find this number
        public static   double SCORE_POSITION = 0;//TODO: find this number
        public static   double SCORE_SPEED = -0.5;//TODO: find this number
        public static   double INTAKE_POSITION = 0.6;//TODO: find this number
        public static   double INTAKE_SPEED = -0.2;//TODO: find this number
    }
    /**
     * Auto constants needed for auto
     * */
    static class AutoConstants{
        public static   double[] SCORE_POS = {0,0,0};
    }
    /**
     * Teleop constants needed for teleop
     * */
    class TeleopConstants{

    }
}
