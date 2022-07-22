package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
//import jdk.nashorn.internal.ir.TryNode;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    // private final CANEncoder driveEncoder;
    // private final CANEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    // private CANCoder canCoder;
    private Rotation2d offset;

    private int tMotorID;
    private int dMotorID;

    public SwerveModule(int dMotorID, int tMotorID, boolean driveMotorReversed, boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {


        driveMotor = new TalonFX(dMotorID);
        turningMotor = new TalonFX(tMotorID);

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        this.dMotorID = dMotorID;
        this.tMotorID = tMotorID;




        // driveEncoder = driveMotor.getEncoder();
        // turningEncoder = turningMotor.getEncoder();

        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition(0);
    }

    public double getTurningPosition() {
        double turningPosition = ((turningMotor.getSelectedSensorPosition(0)/2048) * 2 * Math.PI)/(157/1);
        SmartDashboard.putNumber("Turning Position in Radians", turningPosition);
        return ((turningMotor.getSelectedSensorPosition(0)/2048) * 2 * Math.PI)/(157/1);
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteEncoderRad() {
        //double angle = absoluteEncoder.getAbsolutePosition();
        double angle = absoluteEncoder.getPosition();           //Returns Degrees
        angle *= Math.PI/180;
        angle -= absoluteEncoderOffsetRad;
        angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        //SmartDashboard.putNumber("abs enc", angle);
        System.out.println("abs enc " + angle );
        return angle;
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
        //turningMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        double turningPIDCalculation = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        turningMotor.set(TalonFXControlMode.PercentOutput, turningPIDCalculation);
        if (tMotorID == DriveConstants. kFrontLeftTurningMotorPort)
        {
            SmartDashboard.putNumber("abs enc", getAbsoluteEncoderRad());
            SmartDashboard.putNumber("target setpoint", state.angle.getDegrees());
            SmartDashboard.putNumber("current angle", getTurningPosition() * 180/Math.PI);
            SmartDashboard.putNumber("turning pid controller calculation", turningPIDCalculation);
        }
        //SmartDashboard.putString("Swerve[" + ((AnalogInput) absoluteEncoder).getChannel() + "] state", state.toString());
    }

    public void setAngle(double angle) {
        turningMotor.set(TalonFXControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), angle));
        System.out.println("setAngle called");
    }

    public void stop() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turningMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
}
