package project;

import java.awt.Graphics;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import javax.swing.JPanel;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.tracking.Tracker;
import org.opencv.tracking.TrackerCSRT;
import org.opencv.tracking.TrackerKCF;
import org.opencv.tracking.TrackerMIL;
import org.opencv.tracking.TrackerMOSSE;
import org.opencv.tracking.TrackerTLD;

import javafx.geometry.Point3D;

public class ImagePanel extends JPanel{
	int INDEX;
	CameraManager CM;
	int SOURCE;
	String NAME;
	String TRACKER;
	boolean drawRect;
	boolean trackerInit;
	Pipeline pipeline;
	double height;
	double width;
	double startTime;
	double iterations;
	Rect2d trackingRect;
	
	public ImagePanel(int index, CameraManager cm, int source, String name, String tracker) {
		CM = cm;
		INDEX = index;
		SOURCE = source;
		NAME = name;
		TRACKER = tracker;
		trackingRect = new Rect2d();
		CM.addCamera(INDEX, SOURCE, NAME, TRACKER);
		pipeline = new Pipeline();
		drawRect = false;
		trackerInit = false;
		startTime = (System.currentTimeMillis()/1000);
		iterations = 0;
		myMouseListener listener = new myMouseListener(); 
        addMouseListener(listener);
        addMouseMotionListener(listener);
	}
	
class myMouseListener extends MouseAdapter{
	 @Override
	public void mousePressed(MouseEvent e) {
		 if(CM.currentCamera == INDEX) {
         trackingRect.x = e.getX();
         trackingRect.y = e.getY();
         drawRect = false;
         CM.setTrackingRect(INDEX,trackingRect);
		 }
     }
	 @Override
	public void mouseDragged(MouseEvent e) { 
		 if(CM.currentCamera == INDEX) {
         trackingRect.width = e.getX() - trackingRect.x;
         trackingRect.height = e.getY() - trackingRect.y;
         drawRect = true;
         CM.setTrackingRect(INDEX,trackingRect);
		 }
     }
	 @Override
	public void mouseReleased(MouseEvent e) {
		 if(CM.currentCamera == INDEX) {
		 System.out.println("Camera "+INDEX+" PRESSED");
		 trackingRect.width = e.getX() - trackingRect.x;
         trackingRect.height = e.getY() - trackingRect.y;
         if(!trackerInit && CM.Cameras.get(INDEX).getCalibrationStatus()) {
        	 CM.setTrackingRect(INDEX,trackingRect);
        	 CM.initializeCamera(INDEX);
        	 trackerInit = true;
         }
         drawRect = true;
       }
	 }
}
   

    @Override
    public void paintComponent(Graphics g) {
    	
    	super.paintComponent(g);
    	CM.update();
    	Mat image = CM.Cameras.get(INDEX).getCapturedMat();
    	
    	if(drawRect) {
        	if(trackerInit) {
        		
        		if(!CM.Cameras.get(INDEX).getTrackingStatus()) {
        			trackerInit = false;
        		}else {
        			//System.out.println(CM.calculatePositionUsingCameras());
        		}
        	}
        	Imgproc.rectangle (image,CM.TrackingRects.get(INDEX).tl(),CM.TrackingRects.get(INDEX).br(),new Scalar(0, 0, 255),5);
        }
    	
    	if(CM.Cameras.get(INDEX).getCapturedMat().width() > 0) {
    		g.drawImage(CM.Cameras.get(INDEX).getImage(image), 0, 0, this);
    	} else {
    		g.drawString("CAMERA NOT FOUND", 100, 100);
    	}
        
    }

}