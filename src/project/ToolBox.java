package project;

import java.util.ArrayList;
import java.util.Arrays;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.QRDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;
import org.opencv.core.Point;

import javafx.geometry.Point3D;

public class ToolBox {
	ArrayList<ArrayList> calibrationMatrix;
	ArrayList<Double> calibrationVector;
	ArrayList<ArrayList> projectileMatrix;
	ArrayList<Double> projectileVector;
	
	public ToolBox() {
		Camera c = new Camera();
		c.setX(0);
		c.setY(0);
		c.setZ(0);
		c.setBR(new Point3D(2,2,2));
		c.setTL(new Point3D(-1,1,1));
		c.orthogonalizeFrustum();
		System.out.println(c.getBR());
		System.out.println(c.getTL());
		/*calibrationMatrix = new ArrayList();
		calibrationVector = new ArrayList();
		double[][] XYZD = new double[][] {{1,1,2,0.8},{-1,1,0,Math.pow(5, 0.5001)},{1,-1,0,Math.pow(5, 0.5)},{-1,-1,0,3}};
		//addCalibrationEquationsFromXYZD(XYZD);
		double[] position = calculatePositionFromXYZD(XYZD, Math.PI/4, 0);
		//double[] newVector = rotateAroundX(new double[] {0,1,0}, Math.PI/4);
		System.out.println(position[0]);
		System.out.println(position[1]);
		System.out.println(position[2]);
		System.out.println(calibrationMatrix);
		System.out.println(calibrationVector);*/
	}
	public double[] calculatePositionFromXYZD(double[][] array, double thetaX, double thetaY) {
		double[][] newArray = basisChangeXYZD(array, thetaX, thetaY);
		addCalibrationEquationsFromXYZD(newArray);
		double[] position = solveCalibrationEquations().toArray();
		position[0] = position[0]+newArray[0][0];
		position[1] = position[1]+newArray[0][1];
		position[2] = position[2]+newArray[0][2];
		double[] position2 = rotateAroundY(position, -thetaY);
		double[] position3 = rotateAroundX(position2, -thetaX);
		return position3;
	}
	public RealVector rotateAroundX(RealVector vector, double theta) {
		double[][] matrixData = { {1,0,0}, {0,Math.cos(theta),-Math.sin(theta)}, {0,Math.sin(theta),Math.cos(theta)}};
		RealMatrix matrix = new Array2DRowRealMatrix(matrixData);
		RealVector rotatedVector = matrix.operate(vector);
		return rotatedVector;	
	}
	public double[] rotateAroundX(double[] vector, double theta) {
		double[][] matrixData = { {1,0,0}, {0,Math.cos(theta),-Math.sin(theta)}, {0,Math.sin(theta),Math.cos(theta)}};
		RealMatrix matrix = new Array2DRowRealMatrix(matrixData);
		double[] rotatedVector = matrix.operate(vector);
		return rotatedVector;	
	}
	public RealVector rotateAroundY(RealVector vector, double theta) {
		double[][] matrixData = { {Math.cos(theta),0,Math.sin(theta)}, {0,1,0}, {-Math.sin(theta),0,Math.cos(theta)}};
		RealMatrix matrix = new Array2DRowRealMatrix(matrixData);
		RealVector rotatedVector = matrix.operate(vector);
		return rotatedVector;	
	}
	public double[] rotateAroundY(double[] vector, double theta) {
		double[][] matrixData = { {Math.cos(theta),0,Math.sin(theta)}, {0,1,0}, {-Math.sin(theta),0,Math.cos(theta)}};
		RealMatrix matrix = new Array2DRowRealMatrix(matrixData);
		double[] rotatedVector = matrix.operate(vector);
		return rotatedVector;	
	}
	public int[][] twoLayerTreeHelper(int upperBound){
		int[][] array = new int[upperBound*(upperBound-1)/2][2];
		int x = 0;
			for (int first=0;first<upperBound;first++) {
				for(int second=first+1;second<upperBound;second++) {
					array[x][0] = first;
					array[x][1] = second;
					x++;
				}
			}
		return array;
	}
	public double[][] basisChangeXYZD(double[][] array, double thetaX, double thetaY){
		int x = 0;
		while(x < array.length) {
			double[] newCoordinates = rotateAroundX(new double[]{array[x][0], array[x][1], array[x][2]}, thetaX);
			double[] newCoordinates2 = rotateAroundY(newCoordinates, thetaY);
			array[x][0] = newCoordinates2[0];
			array[x][1] = newCoordinates2[1];
			array[x][2] = newCoordinates2[2];
			x++;
		}
		return array;
	}
	public double[][] calculateEqualExpressionsFromXYZD(double[][] array){
		double[][] eeArray = new double[array.length][4];
		int x = 0;
		while(x<array.length) {
			eeArray[x][0] = 2*array[x][0];
			eeArray[x][1] = 2*array[x][1];
			eeArray[x][2] = 2*array[x][2];
			eeArray[x][3] = (Math.pow(array[x][3],2)-Math.pow(array[x][2],2)-Math.pow(array[x][1],2)-Math.pow(array[x][0],2));
			x++;
		}
		return eeArray;
	}
	public void addCalibrationEquationsFromEqualExpressions(double[][] array) {
		int[][] helper = twoLayerTreeHelper(array.length);
		double[][] matrixExpressions = new double[helper.length][array[0].length - 1];
		double[] vectorConstants = new double[helper.length];
		for(int x = 0; x < helper.length; x++) {
			vectorConstants[x] = array[helper[x][1]][array[0].length-1] - array[helper[x][0]][array[0].length-1];
			for(int y = 0; y < (matrixExpressions[0].length); y++) {
				matrixExpressions[x][y] = array[helper[x][0]][y] - array[helper[x][1]][y];
			}
		}
		int x = 0;
		while(x < helper.length) {
			addCalibrationEquation(matrixExpressions[x], vectorConstants[x]);
			x++;
		}
	}
	public void addCalibrationEquationsFromXYZD(double[][] array) {
		double[][] matrixExpressions = new double[array.length - 1][3];
		double[] vectorConstants = new double[array.length - 1];
		for(int row = 1; row <= matrixExpressions.length; row++) {
			vectorConstants[row - 1] = calculateBij(array, row , 0);
			for(int column = 0; column < 3; column++) {
				matrixExpressions[row - 1][column] = array[row][column] - array[0][column];
			}
		}
		int x = 0;
		while(x < matrixExpressions.length) {
			addCalibrationEquation(matrixExpressions[x], vectorConstants[x]);
			x++;
		}
	}
	public double calculateBij(double[][] array, int i, int j) {
		double Bij = 0.5*(Math.pow(array[j][3],2) - 
					Math.pow(array[i][3],2) +
					(Math.pow((array[i][0]-array[j][0]), 2) +
					Math.pow((array[i][1]-array[j][1]), 2) +
					Math.pow((array[i][2]-array[j][2]), 2)));
		return Bij;
	}
	public double[] arrayList2array(ArrayList arraylist){
		double[] array = new double[arraylist.size()];
		for (int column=0;column<arraylist.size();column++) {
				array[column] = (double)arraylist.get(column);	
		}
		return array;
	}
	public double[][] arrayList2array2D(ArrayList<ArrayList> arraylist){
		double[][] array = new double[arraylist.size()][arraylist.get(0).size()];
		for (int column=0;column<arraylist.size();column++) {
			for(int row=0;row<arraylist.get(0).size();row++) {
				array[column][row] = (double)arraylist.get(column).get(row);
			}
		}
		return array;
	}
	public void resetCalibrationEquations() {
		calibrationMatrix = new ArrayList();
		calibrationVector = new ArrayList();
	}
	public void addCalibrationEquation(double[] matrixExpression, double vectorConstant) {
		ArrayList<Double> cM = new ArrayList();
		for(int x=0;x<matrixExpression.length;x++) {
		cM.add(matrixExpression[x]);
		}
		calibrationMatrix.add(cM);
		calibrationVector.add(vectorConstant);
	}
	public RealVector solveCalibrationEquations() {
		DecompositionSolver solver = 
		new SingularValueDecomposition(new Array2DRowRealMatrix(arrayList2array2D(calibrationMatrix),false)).getSolver();
		RealVector solution = solver.solve(new ArrayRealVector(arrayList2array(calibrationVector), false));
		return solution;
	}
	public void resetProjectileEquations() {
		projectileMatrix = new ArrayList();
		projectileVector = new ArrayList();
	}
	public void addProjectileEquation(double[] matrixExpression, double vectorConstant) {
		ArrayList<Double> pM = new ArrayList();
		for(int x=0;x<matrixExpression.length;x++) {
		pM.add(matrixExpression[x]);
		}
		projectileMatrix.add(pM);
		projectileVector.add(vectorConstant);
	}
	public RealVector solveProjectileEquations() {
		DecompositionSolver solver = 
		new SingularValueDecomposition(new Array2DRowRealMatrix(arrayList2array2D(projectileMatrix),false)).getSolver();
		RealVector solution = solver.solve(new ArrayRealVector(arrayList2array(projectileVector), false));
		return solution;
	}

	
}
