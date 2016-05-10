package com.primesense.nite.Samples.HandViewer;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.List;
import javax.swing.JFrame;

import org.openni.Device;
import org.openni.OpenNI;

import org.openni.*;
import com.primesense.nite.*;

import javax.swing.JOptionPane;

public class HandViewerApplication {

    private JFrame mFrame;
    private HandViewer mViewer;
    private boolean mShouldRun = true;

    public HandViewerApplication(HandTracker tracker) {
        mFrame = new JFrame("NiTE Hand Tracker Viewer");
        mViewer = new HandViewer(tracker);
        
        // register to key events
        mFrame.addKeyListener(new KeyListener() {
            @Override
            public void keyTyped(KeyEvent arg0) {}
            
            @Override
            public void keyReleased(KeyEvent arg0) {}
            
            @Override
            public void keyPressed(KeyEvent arg0) {
                if (arg0.getKeyCode() == KeyEvent.VK_ESCAPE) {
                    mShouldRun = false;
                }
            }
        });
        
        // register to closing event
        mFrame.addWindowListener(new WindowAdapter() {
            public void windowClosing(WindowEvent e) {
                mShouldRun = false;
            }
        });

        mViewer.setSize(800, 600);
        mFrame.add("Center", mViewer);
        mFrame.setSize(mViewer.getWidth(), mViewer.getHeight());
        mFrame.setVisible(true);
    }

    void run() {
        while (mShouldRun) {
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        mFrame.dispose();
    }

    public static void main(String s[]) {
        // initialize OpenNI and NiTE
    	OpenNI.initialize();
        NiTE.initialize();
        
        List<DeviceInfo> devicesInfo = OpenNI.enumerateDevices();
        if (devicesInfo.size() == 0) {
            JOptionPane.showMessageDialog(null, "No device is connected", "Error", JOptionPane.ERROR_MESSAGE);
            return;
        }
        
        Device device = Device.open(devicesInfo.get(0).getUri());
        HandTracker tracker = HandTracker.create();
        tracker.startGestureDetection(GestureType.CLICK);
        tracker.startGestureDetection(GestureType.WAVE);

        final HandViewerApplication app = new HandViewerApplication(tracker);
        app.run();
    }
}
