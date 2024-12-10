package robot.drive;


public class DriveConstants {

    // Wheel and gearing constants
    public static final double WHEEL_RADIUS = 0.08; 
    public static final double CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; 
    public static final double GEARING = 8.0; 

    public static final double POSITION_FACTOR = CIRCUMFERENCE * GEARING; 
    public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0; 

    // Feedforward constants
    public static final class FF {
        public static final double kS = 1.0;  
        public static final double kV = 3.0;  
    }

    // PID constants
    public static final class PID {
        public static final double kP = 8.5; 
        public static final double kI = 0.0;  
        public static final double kD = 0.0; 
        public static final double MAX_SPEED = 2.0; // Meters per second
    }

public static final class otherstuff {
    public static final double TRACK_WIDTH = 0.7112; 
    public static final double MOI = 7.5; 
    public static final double DRIVE_MASS = 60.0; 
    public static final Matrix<N7, N1> STD_DEVS = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0); 
}
