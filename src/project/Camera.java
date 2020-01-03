package project;

import java.awt.Image;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.videoio.Videoio;

import javafx.geometry.Point3D;

public class Camera {
	String TRACKER;
	boolean TrackingSTATUS = false;
	String NAME;
	VideoCap VC;
	double X;
	double Y;
	double Z;
	double CC = -1;
	double Exposure = -6;
	Point3D TL = new Point3D(0,0,0);
	Point3D TR = new Point3D(0,0,0);
	Point3D BL = new Point3D(0,0,0);
	Point3D BR = new Point3D(0,0,0);
	Point3D FP = new Point3D(0,0,0);
	
	public void setCC(double cc) {
		 CC = cc;
	}public double getCC() {
		return CC;
	}public void setExposure(double exposure) {
		 Exposure = exposure;
		 VC.cap.set(Videoio.CAP_PROP_EXPOSURE, exposure);
	}public double getExposure() {
		return Exposure;
	}public void setTracker(String tracker) {
		TRACKER = tracker;
	}public String getTracker() {
		return TRACKER;
	}public void setTrackingStatus(boolean status) {
		TrackingSTATUS = status;
	}public boolean getTrackingStatus() {
		return TrackingSTATUS;
	}public boolean getCalibrationStatus() {
		if((TL != null) && (TR != null) && (BL != null) && (BR != null)) {
			return true;
		}else {
			return false;
		}
	}public void setName(String name) {
		NAME = name;
	}public void setVideoCap(VideoCap vc) {
		VC = vc;
	}public void setX(double x) {
		X = x;
	}public double getX() {
		return X;
	}public void setY(double y) {
		Y = y;
	}public double getY() {
		return Y;
	}public void setZ(double z) {
		Z = z;
	}public double getZ() {
		return Z;
	}public void setTL(Point3D tl) {
		TL = tl;
	}public Point3D getTL() {
		return TL;
	}public void setTR(Point3D tr) {
		TR = tr;
	}public void setBL(Point3D bl) {
		BL = bl;
	}public Point3D getBL() {
		return BL;
	}public Point3D getTR() {
		return TR;
	}public void setBR(Point3D br) {
		BR = br;
	}public Point3D getBR() {
		return BR;
	}public Point3D getLocation() {
		return new Point3D(X,Y,Z);
	}public void orthogonalizeFrustum() {
		double distance1 = this.getTL().distance(X,Y,Z);
		double distance2 = this.getTR().distance(X,Y,Z);
		double distance3 = this.getBL().distance(X,Y,Z);
		double distance4 = this.getBR().distance(X,Y,Z);
		double distanceAverage = (distance1+distance2+distance3+distance4)/4;
		Point3D tl = new Point3D(
				(this.getTL().getX()-X)*(distanceAverage/distance1)+X,
				(this.getTL().getY()-Y)*(distanceAverage/distance1)+Y,
				(this.getTL().getZ()-Z)*(distanceAverage/distance1)+Z);
		Point3D tr = new Point3D(
				(this.getTR().getX()-X)*(distanceAverage/distance2)+X,
				(this.getTR().getY()-Y)*(distanceAverage/distance2)+Y,
				(this.getTR().getZ()-Z)*(distanceAverage/distance2)+Z);
		Point3D bl = new Point3D(
				(this.getBL().getX()-X)*(distanceAverage/distance3)+X,
				(this.getBL().getY()-Y)*(distanceAverage/distance3)+Y,
				(this.getBL().getZ()-Z)*(distanceAverage/distance3)+Z);
		Point3D br = new Point3D(
				(this.getBR().getX()-X)*(distanceAverage/distance4)+X,
				(this.getBR().getY()-Y)*(distanceAverage/distance4)+Y,
				(this.getBR().getZ()-Z)*(distanceAverage/distance4)+Z);
		this.setTL(tl);
		this.setTR(tr);
		this.setBL(bl);
		this.setBR(br);	
	}public void setFP(Point p) {
		//double horizontalProportion = p.x/VC.getCapturedMat().width();
		//double verticalProportion = p.y/VC.getCapturedMat().height();
		//double horizontalProportion = circularFisheyeCorrection(p.x,VC.getCapturedMat().width());
		//double verticalProportion = circularFisheyeCorrection(p.y,VC.getCapturedMat().height());
		//double horizontalProportion = cubicFisheyeCorrection(p.x,VC.getCapturedMat().width(), CC);
		//double verticalProportion = cubicFisheyeCorrection(p.y,VC.getCapturedMat().height(), CC);
		double horizontalProportion = otherFisheyeCorrection(p.x,p.y,VC.getCapturedMat().width(), VC.getCapturedMat().height(), CC)[0];
		double verticalProportion = otherFisheyeCorrection(p.x,p.y,VC.getCapturedMat().width(), VC.getCapturedMat().height(), CC)[1];
		
		Point3D topPoint = new Point3D(horizontalProportion*(TR.getX()-TL.getX())+TL.getX(),
				horizontalProportion*(TR.getY()-TL.getY())+TL.getY(),
				horizontalProportion*(TR.getZ()-TL.getZ())+TL.getZ());
		Point3D bottomPoint = new Point3D(horizontalProportion*(BR.getX()-BL.getX())+BL.getX(),
				horizontalProportion*(BR.getY()-BL.getY())+BL.getY(),
				horizontalProportion*(BR.getZ()-BL.getZ())+BL.getZ());
		FP = new Point3D(verticalProportion*(bottomPoint.getX()-topPoint.getX())+topPoint.getX(),
				verticalProportion*(bottomPoint.getY()-topPoint.getY())+topPoint.getY(),
				verticalProportion*(bottomPoint.getZ()-topPoint.getZ())+topPoint.getZ());
	}public Point3D getFP() {
		return FP;
	}public Mat getCapturedMat() {
		return VC.getCapturedMat();
	}public Image getImage(Mat mat) {
		return VC.getImage(mat);
	}public void read() {
		VC.read();
	}public double circularFisheyeCorrection(double x, double dimension) {
		double y;
		if(x < 0) {
			y = 0;
		}else if(x < dimension/2) {
			y = (Math.sqrt(Math.pow(dimension/2, 2) - Math.pow((x-(dimension/2)),2)))/dimension;
		}else if((dimension/2 < x)&&(x < dimension)) {
			y = (-1*Math.sqrt(Math.pow(dimension/2, 2) - Math.pow((x-(dimension/2)),2)) + dimension)/dimension;
		}else {
			y = 1;
		}
		//System.out.println(y);
		return y;
		
	}public double cubicFisheyeCorrection(double x, double dimension, double c) {
		double y = (-1 - c)/3 * Math.pow(x/dimension, 3) +
				(1 + c)/2 * Math.pow(x/dimension, 2) +
				(5 - c)/6 * Math.pow(x/dimension, 1);
		//System.out.println(y);
		return y;
	}public double[] otherFisheyeCorrection(double x, double y, double width, double height, double c) {
		double[] coordinate = new double[2];
		double radius = Math.pow(Math.pow(x-width/2, 2)+Math.pow(y-height/2, 2), 0.5);
		double standardizedRadius = radius/Math.pow(Math.pow(width/2, 2)+Math.pow(y-height/2, 2), 0.5);
		double newStandardizedRadius = 1 - Math.pow(1 - Math.pow(standardizedRadius, c), 1/c);
		double newRadius = radius*newStandardizedRadius/standardizedRadius;
		coordinate[0] = ((x - width/2)*newRadius/radius + width/2)/width;
		coordinate[1] = ((y - height/2)*newRadius/radius + height/2)/height;
		return coordinate;
	}
	
	
}
