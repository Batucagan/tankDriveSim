// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(Constants.kLeftMotorId, Constants.kMotorType);
  private final CANSparkMax rightMotor = new CANSparkMax(Constants.kRightMotorId, Constants.kMotorType);

  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotor, rightMotor);
  private final AHRS gyro = new AHRS(Port.kMXP,(byte) 60);

  //drive kinetics initialization
  DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(0.75);
  DifferentialDriveWheelSpeeds wheelSpeeds;
  ChassisSpeeds chasisSpeeds;
  double linearSpeed, angularSpeed;

  private Field2d field = new Field2d();

  DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(1),
    7.29,
    7.5,
    20, //! rough estimation of empty chasis with electornics only
    Units.inchesToMeters(6.0),
    Units.inchesToMeters(27.0),
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)); 
     
  //odomatry initialization
  DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(),
    leftMotor.getEncoder().getPosition(),
    rightMotor.getEncoder().getPosition(),
    new Pose2d(8.25, 4.0, new Rotation2d())
    );

  //differential drive simulator initialization


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    leftMotor.getEncoder().setPositionConversionFactor(2 * Math.PI * 6 / 2048);
    rightMotor.getEncoder().setPositionConversionFactor(2 * Math.PI * 6 / 2048);

    REVPhysicsSim.getInstance().addSparkMax(leftMotor, DCMotor.getNEO(1));

    SmartDashboard.putData("field", field);
  }

  public void drive(DoubleSupplier speed, DoubleSupplier rotation, boolean allowTurningPlace) {
    m_drive.curvatureDrive(speed.getAsDouble(), rotation.getAsDouble(), allowTurningPlace);
  }

  @Override
  public void periodic() {
    //set wheel speeds based on encoders on motors
    wheelSpeeds = new DifferentialDriveWheelSpeeds(leftMotor.getEncoder().getPosition(), rightMotor.getEncoder().getPosition());
    //set chasis speed based on speed from wheel speeds
    chasisSpeeds = driveKinematics.toChassisSpeeds(wheelSpeeds);

    //calculate linear and angular speed based on chasis speeds
    linearSpeed = chasisSpeeds.vxMetersPerSecond;
    angularSpeed = chasisSpeeds.omegaRadiansPerSecond;

    var gyroAngle = gyro.getRotation2d();
    driveOdometry.update(
      gyroAngle,
      leftMotor.getEncoder().getPosition(),
      rightMotor.getEncoder().getPosition());
      leftMotor.set(1);
      System.out.println("LeftMotor  : " + leftMotor.get());
      System.out.println("RightMotor : " + rightMotor.get());

      driveOdometry.update(
        gyro.getRotation2d(),
        leftMotor.getEncoder().getPosition(),
        rightMotor.getEncoder().getPosition());
      field.setRobotPose(driveOdometry.getPoseMeters());
  }
  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();

    //? inputs
    driveSim.setInputs(
    leftMotor.get() * RobotController.getInputVoltage(),
    rightMotor.get() * RobotController.getInputVoltage());

    driveSim.update(0.02);

    leftMotor.getEncoder().setPosition(driveSim.getLeftPositionMeters());
    leftMotor.getEncoder().setVelocityConversionFactor(driveSim.getLeftVelocityMetersPerSecond());

    rightMotor.getEncoder().setPosition(driveSim.getRightPositionMeters());
    rightMotor.getEncoder().setVelocityConversionFactor(driveSim.getRightVelocityMetersPerSecond());

    gyro.setAngleAdjustment(-driveSim.getHeading().getDegrees());
  }
}
