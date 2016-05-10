package com.primesense.nite.Samples.HandViewer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.awt.*;
import java.awt.image.*;

import org.openni.*;
import com.primesense.nite.*;

public class HandViewer extends Component 
                        implements HandTracker.NewFrameListener {
    
    float mHistogram[];
    int[] mDepthPixels;
    HandTracker mTracker;
    HandTrackerFrameRef mLastFrame;
    BufferedImage mBufferedImage;

    public HandViewer(HandTracker tracker) {
    	mTracker = tracker;
        mTracker.addNewFrameListener(this);
    }
    
    public synchronized void paint(Graphics g) {
        if (mLastFrame == null) {
            return;
        }
        
        int framePosX = 0;
        int framePosY = 0;
        
        VideoFrameRef depthFrame = mLastFrame.getDepthFrame();
        if (depthFrame != null) {
	        int width = depthFrame.getWidth();
	        int height = depthFrame.getHeight();
	        
	        // make sure we have enough room
	        if (mBufferedImage == null || mBufferedImage.getWidth() != width || mBufferedImage.getHeight() != height) {
	            mBufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
	        }
	        
	        mBufferedImage.setRGB(0, 0, width, height, mDepthPixels, 0, width);
	        
	        framePosX = (getWidth() - width) / 2;
	        framePosY = (getHeight() - height) / 2;

	        g.drawImage(mBufferedImage, framePosX, framePosY, null);
        }
        
        // draw hands
        for (HandData hand : mLastFrame.getHands()) {
        	if (hand.isTracking()) {
        		com.primesense.nite.Point2D<Float> pos = mTracker.convertHandCoordinatesToDepth(hand.getPosition());
        		g.drawRect(framePosX + pos.getX().intValue() - 3, framePosY + pos.getY().intValue() - 3, 5, 5);
        	}
        }
    }

    public synchronized void onNewFrame(HandTracker tracker) {
        if (mLastFrame != null) {
            mLastFrame.release();
            mLastFrame = null;
        }
        
        mLastFrame = mTracker.readFrame();
        
        // check if any gesture detected
        for (GestureData gesture : mLastFrame.getGestures()) {
        	if (gesture.isComplete()) {
        		// start hand tracking
        		mTracker.startHandTracking(gesture.getCurrentPosition());
        	}
        }

        VideoFrameRef depthFrame = mLastFrame.getDepthFrame();
        if (depthFrame != null) {
        	ByteBuffer frameData = depthFrame.getData().order(ByteOrder.LITTLE_ENDIAN);
        
	        // make sure we have enough room
	        if (mDepthPixels == null || mDepthPixels.length < depthFrame.getWidth() * depthFrame.getHeight()) {
	        	mDepthPixels = new int[depthFrame.getWidth() * depthFrame.getHeight()];
	        }
        
            calcHist(frameData);
            frameData.rewind();
            int pos = 0;
            while(frameData.remaining() > 0) {
                short depth = frameData.getShort();
                short pixel = (short)mHistogram[depth];
                mDepthPixels[pos] = 0xFF000000 | (pixel << 16) | (pixel << 8);
                pos++;
            }
        }

        repaint();
    }

    private void calcHist(ByteBuffer depthBuffer) {
        // make sure we have enough room
        if (mHistogram == null) {
            mHistogram = new float[10000];
        }
        
        // reset
        for (int i = 0; i < mHistogram.length; ++i)
            mHistogram[i] = 0;

        int points = 0;
        while (depthBuffer.remaining() > 0) {
            int depth = depthBuffer.getShort() & 0xFFFF;
            if (depth > 9999)
                System.out.println("here: " + depth);
            if (depth != 0) {
                mHistogram[depth]++;
                points++;
            }
        }

        for (int i = 1; i < mHistogram.length; i++) {
            mHistogram[i] += mHistogram[i - 1];
        }

        if (points > 0) {
            for (int i = 1; i < mHistogram.length; i++) {
                mHistogram[i] = (int) (256 * (1.0f - (mHistogram[i] / (float) points)));
            }
        }
    }
}
