package robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Ports;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;  
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DCMotor;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Drive {

    private final CANSparkMax LEFT_FRONT = new CANSparkMax(Ports.Drive.LEFT_FRONT, MotorType.kBrushless);
    private final CANSparkMax LEFT_BACK = new CANSparkMax(Ports.Drive.LEFT_BACK, MotorType.kBrushless);
    private final CANSparkMax RIGHT_FRONT = new CANSparkMax(Ports.Drive.RIGHT_FRONT, MotorType.kBrushless);
    private final CANSparkMax RIGHT_BACK = new CANSparkMax(Ports.Drive.RIGHT_BACK, MotorType.kBrushless);

    // Added the AnalogGyro instance
    private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);

    // Declares the DifferentialDriveOdometry object
    private final DifferentialDriveOdometry odometry; 

    // Feedforward and PID controllers
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.FF.kS, DriveConstants.FF.kV); 
    private final PIDController leftPIDController = new PIDController(DriveConstants.PID.kP, DriveConstants.PID.kI, DriveConstants.PID.kD);
    private final PIDController rightPIDController = new PIDController(DriveConstants.PID.kP, DriveConstants.PID.kI, DriveConstants.PID.kD);

    // Encoder instances (these need to be declared and initialized appropriately)
    private final Encoder leftEncoder = new Encoder(Ports.Drive.LEFT_ENCODER_A, Ports.Drive.LEFT_ENCODER_B);
    private final Encoder rightEncoder = new Encoder(Ports.Drive.RIGHT_ENCODER_A, Ports.Drive.RIGHT_ENCODER_B);

    // Simulation object for the drivetrain
    private final DifferentialDrivetrainSim driveSim;

    public Drive() {
        // Resets the motors to default
        for (CANSparkMax spark : List.of(LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK)) {
            spark.restoreFactoryDefaults();
            spark.setIdleMode(IdleMode.kBrake); 
        }

        // Sets followers to follow the respective leaders
        LEFT_BACK.follow(LEFT_FRONT);
        RIGHT_BACK.follow(RIGHT_FRONT);

        // Inverts the left side motors
        LEFT_FRONT.setInverted(true); 
        LEFT_BACK.setInverted(true);  

        // Sets the encoder conversion factors
        leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
        rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
        leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
        rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);

        // Resets the encoders and gyro
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        gyro.reset();  // Resets the gyro heading to 0

        // Initialize the DifferentialDriveOdometry
        odometry = new DifferentialDriveOdometry(
            new Rotation2d(),  // heading 
            0,  // left encoder position 
            0,  // right encoder position 
            new Pose2d()  // initial position 
        );

        // drivetrain simulation thing 
        driveSim =
            new DifferentialDrivetrainSim(
                DCMotor.getMiniCIM(2), 
                DriveConstants.GEARING,
                DriveConstants.MOI,
                DriveConstants.DRIVE_MASS,
                DriveConstants.WHEEL_RADIUS,
                DriveConstants.TRACK_WIDTH,
                DriveConstants.STD_DEVS
            );
    }

    // Method to drive the robot using PID and Feedforward control
    public void drive(double leftSpeed, double rightSpeed) {
        // Converts the input speeds by the MAX_SPEED
        final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
        final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;
        
        // Calculates feedforward for each side
        final double leftFeedforward = feedforward.calculate(realLeftSpeed);
        final double rightFeedforward = feedforward.calculate(realRightSpeed);

        // Calculates PID for each side
        final double leftPID = leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
        final double rightPID = rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);

        // Combines feedforward and PID to get the final motor voltage
        final double leftVoltage = leftPID + leftFeedforward;
        final double rightVoltage = rightPID + rightFeedforward;

        // Sets the motor voltages based on the calculated voltages
        LEFT_FRONT.setVoltage(leftVoltage);
        RIGHT_FRONT.setVoltage(rightVoltage);
        
        // Updates simulation with applied voltages
        driveSim.setInputs(leftVoltage, rightVoltage);
    }

    // Drive Command Factory
    public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
        return run(() -> drive(vLeft.getAsDouble(), vRight.getAsDouble()));
    }

    // Method to update the odometry
    public void updateOdometry() {
        // Updates the odometry with the current encoder positions and gyro angle
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    // Method to reset the odometry
    public void resetOdometry() {

        // Resets odometry to the initial position
        odometry.resetPosition(new Pose2d(), new Rotation2d());  // Resets to (0, 0) position with 0 heading
        leftEncoder.setPosition(0);  // Resets the left encoder
        rightEncoder.setPosition(0);  // Resets the right encoder
        gyro.reset();  // Resets the gyro heading
    }

    // Method to get the current position of the robot
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // Periodic method to update odometry every tick
    public void periodic() {
        // Updates odometry 
        updateOdometry(Robot.isReal() ? gyro.getRotation2d() : driveSim.getHeading());
    }

    @Override
    public void simulationPeriodic() {
        // sim.update() tells the simulation how much time has passed
        driveSim.update(Constants.PERIOD.in(Seconds));

        // Update encoder positions based on the simulated drivetrain
        leftEncoder.setPosition(driveSim.getLeftPositionMeters());
        rightEncoder.setPosition(driveSim.getRightPositionMeters());

    } 
