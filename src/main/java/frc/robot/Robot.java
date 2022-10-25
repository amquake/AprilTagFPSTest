// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {

    private final Field2d field = new Field2d();

    private final String cameraName = "mmal_service_16.1";
    private PhotonCamera camera;
    private NetworkTableInstance instance;

    // average ms vetween frames
    private LinearFilter actualFilter = LinearFilter.movingAverage(40);
    double lastTime = 0;

    DataLog log;
    DoubleLogEntry logDiff;
    DoubleLogEntry logDiffAvg;

    @Override
    public void robotInit() {
        instance = NetworkTableInstance.getDefault();
        instance.stopServer();
        instance.startClient("photonvision.local");
        
        camera = new PhotonCamera(instance, cameraName);
        
        SmartDashboard.putData(field);
        field.setRobotPose(-100, -100, new Rotation2d());

        DataLogManager.logNetworkTables(false);
        log = DataLogManager.getLog();
        logDiff = new DoubleLogEntry(log, "actualMs");
        logDiffAvg = new DoubleLogEntry(log, "avgActualMs");

        instance.addEntryListener(
            instance.getTable("photonvision").getSubTable(cameraName).getEntry("rawBytes"),
            (notif)->{
                double now = Timer.getFPGATimestamp();
                double diff = now - lastTime;
                double diffAvg = actualFilter.calculate(diff);
                lastTime = now;

                SmartDashboard.putNumber("Diff Ms", diff*1000);
                logDiff.append(diff*1000);

                SmartDashboard.putNumber("Diff Avg Ms", diffAvg*1000);
                logDiffAvg.append(diffAvg*1000);
                SmartDashboard.putNumber("Avg FPS", 1.0/diffAvg);
            },
            EntryListenerFlags.kUpdate
        );
    }
    
    @Override
    public void robotPeriodic() {
    }
    
    @Override
    public void autonomousInit() {}
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {}
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void testInit() {}
    
    @Override
    public void testPeriodic() {}
    
    @Override
    public void simulationInit() {}
    
    @Override
    public void simulationPeriodic() {}
}
