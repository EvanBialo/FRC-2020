package ca.warp7.frc2020.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Infrastructure implements Subsystem {
    private static Infrastructure instance;

    public static Infrastructure getInstance() {
        if (instance == null) {
            instance = new Infrastructure();
        }
        return instance;
    }

    private double dt = 1.0/50; // s
    private double previousTime = 0.0; // s

    @Override
    public void periodic() {
        double time = Timer.getFPGATimestamp();
        dt = time - previousTime;
        previousTime = time;
    }

    private Compressor compressor = new Compressor();

    private Infrastructure() {
    }

    public double getDt(){
        return dt;
    }

    public void startCompressor() {
        compressor.start();
    }

    public void stopCompressor() {
        compressor.stop();
    }
}
