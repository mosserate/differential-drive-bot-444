package robot;

public final class Ports {
  // Add and change all ports as needed.
  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  // made a new class for drive
  public static final class Drive {
    public static final int LEFT_FRONT = 2; // represents location of wheel
    public static final int LEFT_BACK = 3;
    public static final int RIGHT_FRONT = 4;
    public static final int RIGHT_BACK = 5; 

    // Add the gyro channel
    public static final int GYRO_CHANNEL = 1; // Gyro is connected to port 1
  }
}
