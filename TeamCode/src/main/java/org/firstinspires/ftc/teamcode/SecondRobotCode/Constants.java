package org.firstinspires.ftc.teamcode.SecondRobotCode;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class Constants {
    /**
     * All constants needed for the chassis
     * */
    class ChassisConstants{
        public static final int WHEELDIAMATER = 0; //TODO: find this number
        public static final double WHEELCIRCUMFRANCE = WHEELDIAMATER* Math.PI;
        public static final double CHASSISWIDTH = 0; //TODO: find this number
        public static final double CHASSISLENGTH = 0; // TODO: find this number

    }
    /**
     * All constants needed for the the arm
     * */
    class ArmConstants{

        public static final double GEARRATIO = 1; //TODO: get this number

        double KP = 0;
        public static final double TOLERANCE = 0;//TODO: find this number
        public static final double Kp = 0; //TODO: Find P Value
        public static final double Ki = 0; //TODO: Find I Value
        public static final double Kd = 0; //TODO: Find D Value

    }
    public static class ArmExtender {
       public static final double WHEEDIAMITER = 1; //TODO: Find wheel diameter
    }
    /**
     * Auto constants needed for auto
     * */
    class AutoConstants{

    }
    /**
     * Teleop constants needed for teleop
     * */
    class TeleopConstants{

    }
}
