package org.usfirst.frc.team2485.util;

import javafx.geometry.Point2D;

/**
 * @author Ben Dorsey
 */

public class AutoPath {
	private class Point {
		private double x, y; 
		private double heading, curvature;
		private double arcLength;
		private Point(Point2D p) {
			this.x = p.getX();
			this.y = p.getY();
		}
	}
	private Point[] points;
	
	public AutoPath(Point2D[] points) {
		this.points = new Point[points.length];
		for (int i = 0; i < points.length; i++) {
			this.points[i] = new Point(points[i]);
		}
		generateCurve();
	} 
	
	private void generateCurve() {
		int len = points.length;
		points[0].arcLength = 0;
		for (int i = 0; i < len - 1; i++) {
			double dX = points[i + 1].x - points[i].x;
			double dY = points[i + 1].y - points[i].y;
			points[i].heading = Math.atan2(dX, dY); // this is switched intentionally i swear
			points[i + 1].arcLength = points[i].arcLength + Math.hypot(dX, dY);
		}
		points[len - 1].heading = points[len - 1].heading;
		
		for (int i = 0; i < points.length - 2; i++) {
			points[i].curvature = (points[i + 1].heading - points[i].heading) / 
					(points[i + 1].arcLength - points[i].arcLength);
		}
		points[len - 1].curvature = points[len - 2].heading = points[len - 3].heading;
	}
	
	private Point getPointAtDist(double dist) {
		for (int i = 0; i < points.length; i++) {
			if (dist < points[i].arcLength) {
				return points[i];
			}
		}
		return points[points.length - 1];
	}

	public double getCurvatureAtDist(double dist) {
		return getPointAtDist(dist).curvature;
	}
	
	public double getHeadingAtDist(double dist) {
		return getPointAtDist(dist).heading;
	}
	
	public double getPathLength() {
		return points[points.length - 1].arcLength;
	}
}
