package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;

public class PigeonTwo {
    
    //Pigeon instance (ensure it is a singleton)
    private static PigeonTwo instance = null;

    /**
     * Method to get current instance of Pigeon2 (or establish one if one does not exist)
     * @return Pigeon2 Instance
     */
    public static PigeonTwo getInstance(){
        if(instance == null){
            instance = new PigeonTwo();
        }
        return instance;
    }

    //Pigeon2 object
    private Pigeon2 m_pigeon2;

    //Pigeon2 configuration settings
    private Pigeon2Configuration config = new Pigeon2Configuration();

    //Pigeon2 Fault Log
    private Pigeon2_Faults faultLog = new Pigeon2_Faults();

    //Fault ErrorCode object
    ErrorCode faults;

    //GravityVector object
    private double[] a_gravityVector = new double[3];
    
    private PigeonTwo(){
        try{
            m_pigeon2 = new Pigeon2(0); //CHECK PORT AND CONSTRUCTOR
            //Sets the status frame period for two different periods.
            m_pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 5, 10);
            m_pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 5, 10);
            
            //Default configs = MAKE CONSTANTS IN CONSTANTS FILE & DETERMINE VALUES
            config.DisableNoMotionCalibration = false;
            config.DisableTemperatureCompensation = false;
            config.EnableCompass = true;
            config.MountPosePitch = 0;
            config.MountPoseRoll = 0;
            config.MountPoseYaw = 0;

            //Sets pigeon default mountings to values determined above
            m_pigeon2.configAllSettings(config); 

            //Gets gravity vectory and assigns it to a_gravityVector
            m_pigeon2.getGravityVector(a_gravityVector);

            //Assigns faultLog to record errors
            faults = m_pigeon2.getFaults(faultLog);
        }catch(Exception e){
            System.out.println("PIGEON INSTANTATION FAILED");
            m_pigeon2.DestroyObject();
            e.printStackTrace();
        }
    }

    // public boolean isGood(){
    //     return true;
    // }

    public Rotation2d getPitch(){
        double pitch = m_pigeon2.getPitch();
        return Rotation2d.fromDegrees(pitch);
    }

    public Rotation2d getAngle(){
        double yaw = m_pigeon2.getYaw();
        return Rotation2d.fromDegrees(yaw);
    }

    public Rotation2d getRoll(){
        double roll = m_pigeon2.getRoll(); //COMPARE TO getYPR();
        return Rotation2d.fromDegrees(roll);
    }

    public double getRateX(){
        double[] ypr = new double[3];
        m_pigeon2.getRawGyro(ypr);
        return ypr[0];
    }

    public double getRateY(){
        double[] ypr = new double[3];
        m_pigeon2.getRawGyro(ypr);
        return ypr[1];
    }

    public double getRateZ(){
        double[] ypr = new double[3];
        m_pigeon2.getRawGyro(ypr);
        return ypr[2];
    }

    public double getRawAngle(){
        double[] ypr = new double[3];
        m_pigeon2.getYawPitchRoll(ypr);
        return ypr[0];
    }

    public double get360Angle(){
        double[] ypr = new double[3];
        m_pigeon2.getYawPitchRoll(ypr);
        return ypr[0] % 360;
    }

    public ErrorCode setAngle(double degrees){
        return m_pigeon2.setYaw(degrees);
    }

    public ErrorCode reset(){
        return setAngle(0.0);
    }

    public ErrorCode getGravityVector(){
        return m_pigeon2.getGravityVector(a_gravityVector);
    }

    //CODE RELATED TO ERROR MANAGEMENT//

    public ErrorCode getLastError(){
        return m_pigeon2.getLastError();
    }

    public boolean getAPIError(){
        return faultLog.APIError;
    }

    public boolean getAccelError(){
        return faultLog.AccelFault;
    }

    public boolean getBootIntoMotionError(){
        return faultLog.BootIntoMotion;
    }

    public boolean getGyroError(){
        return faultLog.GyroFault;
    }

    public boolean getHardwareError(){
        return faultLog.HardwareFault;
    }

    public boolean getMagnetometerError(){
        return faultLog.MagnetometerFault;
    }

    public boolean getMotionDriverError(){
        return faultLog.BootIntoMotion;
    }

    public boolean getResetError(){
        return faultLog.ResetDuringEn;
    }

    public boolean getSaturatedAccelError(){
        return faultLog.SaturatedAccel;
    }

    public boolean getSaturatedMagError(){
        return faultLog.SaturatedMag;
    }

    public boolean getSaturatedRotVelError(){
        return faultLog.SaturatedRotVelocity;
    }

    public boolean getVoltageError(){
        return faultLog.UnderVoltage;
    }

}