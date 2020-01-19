package project;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Rect2d;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.tracking.*;


import javafx.geometry.Point3D;

public class CameraManager {
	ArrayList<Camera> Cameras = new ArrayList();
	ArrayList<Tracker> Trackers = new ArrayList();
	ArrayList<Rect2d> TrackingRects = new ArrayList();
	ArrayList<MotionTracker> MotionTrackers = new ArrayList();
	
	
	Point center = new Point();
	
	int currentCamera = 0;
	
	public CameraManager() {
	
	}
	
	public void addCamera(int index, int source, String name, String tracker) {
		Camera c = new Camera();
		c.setName(name);
		VideoCap vc = new VideoCap(source);
		c.setVideoCap(vc);
		c.setTracker(tracker);
		Cameras.add(index, c);
		Rect2d tr = new Rect2d();
		TrackingRects.add(index, tr);
		addTracker(index, Cameras.get(index).getTracker());
		addMotionTracker(index);
	}public void addTracker(int cameraNumber, String tracker) {
		if(tracker == "TrackerBoosting") {
			Tracker t = TrackerBoosting.create();
			Trackers.add(cameraNumber, t);
		}else if(tracker == "TrackerCSRT") {
			Tracker t = TrackerCSRT.create();
			Trackers.add(cameraNumber, t);
		}else if(tracker == "TrackerKCF") {
			Tracker t = TrackerKCF.create();
			Trackers.add(cameraNumber, t);
		}else if(tracker == "TrackerMOSSE") {
			Tracker t = TrackerMOSSE.create();
			Trackers.add(cameraNumber, t);
		}else{
			System.out.println("INVALID TRACKER. DEFAULTING TO TrackerCSRT");
			Tracker t = TrackerCSRT.create();
			Trackers.add(cameraNumber, t);
		}
	}public void setTracker(int cameraNumber, String tracker) {
		if(tracker == "TrackerBoosting") {
			Tracker t = TrackerBoosting.create();
			Trackers.set(cameraNumber, t);
		}else if(tracker == "TrackerCSRT") {
			Tracker t = TrackerCSRT.create();
			Trackers.set(cameraNumber, t);
		}else if(tracker == "TrackerKCF") {
			Tracker t = TrackerKCF.create();
			Trackers.set(cameraNumber, t);
		}else if(tracker == "TrackerMOSSE") {
			Tracker t = TrackerMOSSE.create();
			Trackers.set(cameraNumber, t);
		}else{
			System.out.println("INVALID TRACKER. DEFAULTING TO TrackerCSRT");
			Tracker t = TrackerCSRT.create();
			Trackers.set(cameraNumber, t);
		}
	}
	public void setTrackingRect(int cameraNumber, Rect2d tr) {
		TrackingRects.set(cameraNumber, tr);
	}public void addMotionTracker(int cameraNumber) {
		MotionTracker MT = new MotionTracker();
		MotionTrackers.add(cameraNumber, MT);
		}
	public void initializeCamera(int cameraNumber) {
		Cameras.get(cameraNumber).read();
   	 	Mat image = Cameras.get(cameraNumber).getCapturedMat();
   	 	setTracker(cameraNumber, Cameras.get(cameraNumber).getTracker());
   	 	Trackers.get(cameraNumber).init(image, TrackingRects.get(cameraNumber));
   	 	Cameras.get(cameraNumber).setTrackingStatus(true);
	}public void initializeCamera(int cameraNumber, Mat image) {
   	 	setTracker(cameraNumber, Cameras.get(cameraNumber).getTracker());
   	 	Trackers.get(cameraNumber).init(image, TrackingRects.get(cameraNumber));
   	 	Cameras.get(cameraNumber).setTrackingStatus(true);
	}public void update(int cameraNumber) {
		//for(int cameraNumber=0;cameraNumber<Cameras.size();cameraNumber++) {
		Cameras.get(cameraNumber).read();
		Mat image = Cameras.get(cameraNumber).getCapturedMat();
		if(Cameras.get(cameraNumber).TrackingMode == "MOTION DETECTION") {
    	try {
    	MotionTrackers.get(cameraNumber).process(image);
    	Mat img = MotionTrackers.get(cameraNumber).desaturateOutput();
    	int rows = img.rows(); //Calculates number of rows
    	int cols = img.cols(); //Calculates number of columns
    	int ch = img.channels(); //Calculates number of channels (Grayscale: 1, RGB: 3, etc.)
    	double xMoment = 0;
    	double xDistance = 0;
    	double yMoment = 0;
    	double yDistance = 0;
    	double Mass = 0;
    	
    	for (int i=0; i<rows; i++)
    	{
    	    for (int j=0; j<cols; j++)
    	    {
    	        double[] data = img.get(i, j); //Stores element in an array
    	        if (data[0] > 50) {
    	        xMoment += data[0]*j;
    	        yMoment += data[0]*i;
    	        Mass += data[0];
    	        }
    	    }
    	}
    	if(Mass > 0) {
    	Cameras.get(cameraNumber).setTrackingStatus(true);
    	xDistance = xMoment/Mass;
    	yDistance = yMoment/Mass;
    	//System.out.println(Mass);
    	Rect autoTrackingRect = new Rect(new Point(xDistance-20,yDistance-20), new Point(xDistance+20,yDistance+20));
    	
    	Cameras.get(cameraNumber).setAutoTrackingRect(autoTrackingRect);
    	this.center.x = (Cameras.get(cameraNumber).AutoTrackingRect.tl().x + Cameras.get(cameraNumber).AutoTrackingRect.br().x)/2;
		this.center.y = (Cameras.get(cameraNumber).AutoTrackingRect.tl().y + Cameras.get(cameraNumber).AutoTrackingRect.br().y)/2;
		Cameras.get(cameraNumber).setFP(this.center);
    	}else {
    		Rect autoTrackingRect = new Rect();
    		Cameras.get(cameraNumber).setTrackingStatus(false);
    		Cameras.get(cameraNumber).setAutoTrackingRect(autoTrackingRect);
    	}
    	}catch(Exception e) {
    		Cameras.get(cameraNumber).setTrackingStatus(false);
    	}
    	
		
	}else if (Cameras.get(cameraNumber).TrackingMode == "MANUAL TRACKING"){
		if(Cameras.get(cameraNumber).getTrackingStatus() == true) {
			
			
	   		boolean updateCheck = Trackers.get(cameraNumber).update(image, TrackingRects.get(cameraNumber));
    		if(!updateCheck) {
    			Cameras.get(cameraNumber).setTrackingStatus(false);
    			//Cameras.get(cameraNumber).FP = null;
    			System.out.println("PROBLEM WITH CAMERA #"+cameraNumber);
    		}else {
    			
    			center.x = (TrackingRects.get(cameraNumber).tl().x + TrackingRects.get(cameraNumber).br().x)/2;
    			center.y = (TrackingRects.get(cameraNumber).tl().y + TrackingRects.get(cameraNumber).br().y)/2;
    			Cameras.get(cameraNumber).setFP(center);
    			//System.out.println(Cameras.get(cameraNumber).getFP());
    		}
			
		}
	} else if (Cameras.get(cameraNumber).TrackingMode == "MOTION TRACKING"){
			if(Cameras.get(cameraNumber).getTrackingStatus() == true) {
				
				
				
		   		boolean updateCheck = Trackers.get(cameraNumber).update(image, TrackingRects.get(cameraNumber));
	    		if(!updateCheck) {
	    			Cameras.get(cameraNumber).setTrackingStatus(false);
	    			//Cameras.get(cameraNumber).FP = null;
	    			System.out.println("PROBLEM WITH CAMERA #"+cameraNumber);
	    		}else {
	    			
	    			center.x = (TrackingRects.get(cameraNumber).tl().x + TrackingRects.get(cameraNumber).br().x)/2;
	    			center.y = (TrackingRects.get(cameraNumber).tl().y + TrackingRects.get(cameraNumber).br().y)/2;
	    			Cameras.get(cameraNumber).setFP(center);
	    			//System.out.println(Cameras.get(cameraNumber).getFP());
	    		}
				
			}else {
				try {
			    	MotionTrackers.get(cameraNumber).process(image);
			    	Mat img = MotionTrackers.get(cameraNumber).desaturateOutput();
			    	int rows = img.rows(); //Calculates number of rows
			    	int cols = img.cols(); //Calculates number of columns
			    	int ch = img.channels(); //Calculates number of channels (Grayscale: 1, RGB: 3, etc.)
			    	double xMoment = 0;
			    	double xDistance = 0;
			    	double yMoment = 0;
			    	double yDistance = 0;
			    	double Mass = 0;
			    	
			    	for (int i=0; i<rows; i++)
			    	{
			    	    for (int j=0; j<cols; j++)
			    	    {
			    	        double[] data = img.get(i, j); //Stores element in an array
			    	        if (data[0] > 50) {
			    	        xMoment += data[0]*j;
			    	        yMoment += data[0]*i;
			    	        Mass += data[0];
			    	        }
			    	    }
			    	}
			    	if(Mass > 0) {
			    	Cameras.get(cameraNumber).setTrackingStatus(true);
			    	xDistance = xMoment/Mass;
			    	yDistance = yMoment/Mass;
			    	//System.out.println(Mass);
			    	Rect autoTrackingRect = new Rect(new Point(xDistance-60,yDistance-60), new Point(xDistance+20,yDistance+20));
			    	setTrackingRect(cameraNumber,new Rect2d(autoTrackingRect.tl(),autoTrackingRect.br()));
			    	initializeCamera(cameraNumber, image);
			    	//Cameras.get(cameraNumber).setAutoTrackingRect(autoTrackingRect);
			    	
			    	/*this.center.x = (Cameras.get(cameraNumber).AutoTrackingRect.tl().x + Cameras.get(cameraNumber).AutoTrackingRect.br().x)/2;
					this.center.y = (Cameras.get(cameraNumber).AutoTrackingRect.tl().y + Cameras.get(cameraNumber).AutoTrackingRect.br().y)/2;
					Cameras.get(cameraNumber).setFP(this.center);*/
			    	}else {
			    		Rect autoTrackingRect = new Rect();
			    		Cameras.get(cameraNumber).setTrackingStatus(false);
			    		Cameras.get(cameraNumber).setAutoTrackingRect(autoTrackingRect);
			    	}
			    	}catch(Exception e) {
			    		Cameras.get(cameraNumber).setTrackingStatus(false);
			    	}
			}
	}
		
	//}
		
	}public Point3D calculatePositionUsingCameras() {
		ArrayList<Point3D> points = new ArrayList();
		for (int x=1; x<Cameras.size();x++) {
			for (int y=0; y<x;y++) {
				if((Cameras.get(x).getTrackingStatus()) && (Cameras.get(y).getTrackingStatus())) {
					//System.out.println("Point Added");
					points.add(this.findIntersection(Cameras.get(x), Cameras.get(y)));
				}
			}
		}
		
		Point3D sumPoint = new Point3D(0,0,0);
		for (int z=0;z<points.size();z++) {
			sumPoint = sumPoint.add(points.get(z));
		}
		
		if(points.size() > 0) {
			//System.out.println(points.size());
			return points.get(0);
			//return sumPoint.multiply(1/(double)points.size());
		
		}else {
			//System.out.println(points.size());
			return new Point3D(1,1,1);
		}
	}private double dFunc(Point3D p1, Point3D p2, Point3D p3, Point3D p4) {
		return (p1.getX() - p2.getX())*(p3.getX() - p4.getX())+
				(p1.getY() - p2.getY())*(p3.getY() - p4.getY())+
				(p1.getZ() - p2.getZ())*(p3.getZ() - p4.getZ());
	}public Point3D findIntersection(Camera c1, Camera c2) {
		double Ma = ((dFunc(c1.getLocation(),c2.getLocation(),c2.getFP(),c2.getLocation())
				*dFunc(c2.getFP(),c2.getLocation(),c1.getFP(),c1.getLocation()))
				-
				(dFunc(c2.getFP(),c2.getLocation(),c2.getFP(),c2.getLocation())
				*dFunc(c1.getLocation(),c2.getLocation(),c1.getFP(),c1.getLocation())))
				/
				((dFunc(c1.getFP(),c1.getLocation(),c1.getFP(),c1.getLocation())
				*dFunc(c2.getFP(),c2.getLocation(),c2.getFP(),c2.getLocation()))
				-
				(dFunc(c1.getFP(),c1.getLocation(),c2.getFP(),c2.getLocation())
				*dFunc(c2.getFP(),c2.getLocation(),c1.getFP(),c1.getLocation())));
		double Mb = (dFunc(c1.getLocation(), c2.getLocation(), c2.getFP(), c2.getLocation())
				+
				Ma*dFunc(c1.getFP(), c1.getLocation(), c2.getFP(), c2.getLocation()))
				/
				dFunc(c2.getFP(), c2.getLocation(), c2.getFP(), c2.getLocation());
		Point3D Pa = c1.getFP().subtract(c1.getLocation()).multiply(Ma).add(c1.getLocation());
		Point3D Pb = c2.getFP().subtract(c2.getLocation()).multiply(Mb).add(c2.getLocation());
		return Pa.midpoint(Pb);
	}
}
