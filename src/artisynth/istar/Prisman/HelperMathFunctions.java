/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.istar.Prisman;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Scanner;

import org.python.modules.math;

import artisynth.core.mechmodels.FrameMarker;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point2d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class HelperMathFunctions {

   public double getSquareDistance (VectorNd v, VectorNd v2) {

      double dx = v.get (0) - v2.get (0);
      double dy = v.get (1) - v2.get (1);
      double dz = v.get (2) - v2.get (2);

      return dx * dx + dy * dy + dz * dz;
   }

   public double getSquareDistance (Vector3d v, Vector3d v2) {
      return getSquareDistance (new VectorNd (v), new VectorNd (v2));
   }

   public void planeFit (Vector3d[] points, Vector3d center, Vector3d normal) {
      // Points: Input. Array of Point3d objects specifying points to be
      // regressed
      // Center: Output: point on plane
      // Normal: Output. Normal of plane
      double sumX = 0;
      double sumY = 0;
      double sumZ = 0;
      int numberOfNan = 0;

      int n = points.length;
      Vector3d[] copyOfPoints = new Vector3d[n];

      for (int i = 0; i < n; i++) {
         sumX += points[i].get (0);
         sumY += points[i].get (1);
         sumZ += points[i].get (2);
         copyOfPoints[i] = new Vector3d (points[i].copy ());
      }

      center = new Point3d (sumX / n, sumY / n, sumZ / n);

      MatrixNd mat = new MatrixNd (n, 3);

      for (int i = 0; i < n; i++) {

         copyOfPoints[i].sub (center);
         mat.setRow (i, points[i]);
      }

      SVDecomposition svd = new SVDecomposition (mat);
      // MatrixNd V = svd.getV();
      // MatrixNd U = svd.getU ();
      // VectorNd S = svd.getS ();
      svd.getV ().getColumn (2, normal);

   }

   double pointPlaneDistance (Vector3d point, double[] planeValues) {
      // returns the distance from a point to a plane defined as Ax + By + Cz +
      // D = 0
      double numerator =
         Math.abs (
            planeValues[0] * point.get (0) + planeValues[1] * point.get (1)
            + planeValues[2] * point.get (2) + planeValues[3]);
      double denominator =
         Math.sqrt (
            Math.pow (planeValues[0], 2) + Math.pow (planeValues[1], 2)
            + Math.pow (planeValues[2], 2));
      return numerator / denominator;
   }

   double[] planeEquationFromNormalAndPoint (Vector3d normal, Vector3d point) {
      // Ax + By + Cz + D = 0
      double[] planeValues = new double[4];
      for (int j = 0; j < 3; j++) {
         planeValues[j] = normal.get (j);
      }
      planeValues[3] =
         -normal.get (0) * point.get (0) - normal.get (1) * point.get (1)
         - normal.get (2) * point.get (2);
      // returns array of 4: A, B, C, D
      return planeValues;
   }

   public Vector3d[] lineFitThree (Vector3d[] points) {
      // First element in array is the line direction, second element is mean
      Vector3d[] directionAndMean = new Vector3d[2];

      int size = points.length;
      // Setting up matrix for SVD
      MatrixNd svdMatrix = new MatrixNd (size, 3);
      for (int i = 0; i < size; i++) {
         svdMatrix.setRow (i, new Vector3d (points[i]));
      }

      // Calcluating mean value of points
      VectorNd xColumn = new VectorNd ();
      VectorNd yColumn = new VectorNd ();
      VectorNd zColumn = new VectorNd ();

      svdMatrix.getColumn (0, xColumn);
      svdMatrix.getColumn (1, yColumn);
      svdMatrix.getColumn (2, zColumn);

      Vector3d mean =
         new Vector3d (xColumn.mean (), yColumn.mean (), zColumn.mean ());

      // Creating matrix of mean
      MatrixNd svdMatrixMean = new MatrixNd (size, 3);
      for (int i = 0; i < size; i++) {
         svdMatrixMean.setRow (i, mean.copy ());
      }
      // Subtraction mean from matrix based on formula
      svdMatrix.sub (svdMatrixMean);

      SVDecomposition svd = new SVDecomposition (svdMatrix);

      VectorNd fibulaLengthDir = new VectorNd ();
      svd.getV ().getColumn (0, fibulaLengthDir);
      fibulaLengthDir.normalize ();
      directionAndMean[0] = new Vector3d (fibulaLengthDir);
      directionAndMean[1] = new Vector3d (mean);

      return directionAndMean;
   }

   public double ExtensionCalculation (double distance) {
      double ext = (-0.01 * distance * distance) + 10;
      return ext;
   }

   public Vector3d[] lineFitThree (FrameMarker[] framemarkers) {
      int size = framemarkers.length;
      Vector3d[] points = new Vector3d[size];

      for (int i = 0; i < size; i++) {
         points[i] = new Vector3d (framemarkers[i].getLocation ().copy ());
      }

      return lineFitThree (points);
   }

   public double PlanePointDistance (
      Vector3d normal, Vector3d center, Vector3d point) {
      Vector3d v = new Vector3d (point);
      v.sub (center);
      double dot = normal.dot (v);
      dot = dot / (normal.norm ());
      double distance = Math.abs (dot);

      return distance;

   }
   
   public double getAngleBetweenVectors(Vector3d A, Vector3d B) {
      double dot = A.dot (B);
      return math.acos (dot/(A.norm () *B.norm()));
   }
   
   public NumericListKnot closestNumericListKnotToPlane (
      Vector3d normal, Vector3d point, NumericList numericList) {
      // Returns the closest knot in a numericList to a plane defined by a
      // normal and a point, and pose.

      // Transform normal and point so that they are in the correct orientation
      // and location in the world space
      Vector3d worldNormal = new Vector3d (normal);
      // worldNormal.transform (pose);
      // System.out.println (worldNormal);
      Point3d worldPoint = new Point3d (point);
      // worldPoint.transform (pose);

      // Getting plane equation from worldNormal and worldPoint
      double[] planeValues =
         planeEquationFromNormalAndPoint (worldNormal, worldPoint);

      // Setting up variables and iterator used in finding the point on the
      // curved line that is closest to the plane
      double distance = 1000;
      double centerdistance = 1000;
      Vector3d tempPoint;
      NumericListKnot closestKnot = numericList.getLast ();
      Iterator<NumericListKnot> itr = numericList.iterator ();
      NumericListKnot minpointplanedistance = numericList.getLast ();
      NumericListKnot minpointplanedistance2 = numericList.getLast ();
      double threshold = 25; // need to be replaced with actual threshold

      /*
       * while (itr.hasNext ()) { tempPoint = new Point3d (itr.next().v); double
       * currentDistance = pointPlaneDistance(tempPoint, planeValues); if
       * (currentDistance < distance) { distance = currentDistance; closestKnot
       * = itr.next (); } }
       */

      itr = numericList.iterator ();
      // Finding the point where tempPoint is closest to the plane
      while (itr.hasNext ()) {
         tempPoint = new Vector3d (itr.next ().v);
         double currentDistance =
            PlanePointDistance (worldNormal, worldPoint, tempPoint);
         double pointcenterDistance =
            getSquareDistance (new Point3d (tempPoint), worldPoint);
         if ((currentDistance < distance) && (itr.hasNext ())) {
            distance = currentDistance;
            minpointplanedistance = itr.next ();
         }
      }

      itr = numericList.iterator ();
      // Finding a second point that is closes to the plane and on the curve
      // minimum threshold distance away from point minpointplanedistanceidx
      distance = 1000;
      while (itr.hasNext ()) {
         tempPoint = new Vector3d (itr.next ().v);
         double pointpointdis =
            tempPoint.distance (new Vector3d (minpointplanedistance.v));
         if ((pointpointdis > threshold) && (itr.hasNext ())) {
            double pointplanedistance =
               PlanePointDistance (worldNormal, worldPoint, tempPoint);
            if ((pointplanedistance < distance) && (itr.hasNext ())) {
               minpointplanedistance2 = itr.next ();
               distance = pointplanedistance;
            }
         }
      }

      // Checking which of the two points with minimum distance to the plane is
      // closest to the plane's center and set closestKnot accordingly.
      double pointcenterdistance =
         getSquareDistance (minpointplanedistance.v, new VectorNd (worldPoint));
      closestKnot = minpointplanedistance;
      double pointcenterdistance2 =
         getSquareDistance (minpointplanedistance2.v, new VectorNd (worldPoint));
      if (pointcenterdistance > pointcenterdistance2) {
         closestKnot = minpointplanedistance2;
      }

      return closestKnot;
   }

   public NumericListKnot closestNumericListKnotToPlaneMaxilla (
      Vector3d normal, Vector3d point, NumericList numericList) {
      // Returns the closest knot in a numericList to a plane defined by a
      // normal and a point, and pose.

      // Transform normal and point so that they are in the correct orientation
      // and location in the world space
      Vector3d worldNormal = new Vector3d (normal);
      // worldNormal.transform (pose);
      // System.out.println (worldNormal);
      Point3d worldPoint = new Point3d (point);
      // worldPoint.transform (pose);

      // Getting plane equation from worldNormal and worldPoint
      double[] planeValues =
         planeEquationFromNormalAndPoint (worldNormal, worldPoint);

      // Setting up variables and iterator used in finding the point on the
      // curved line that is closest to the plane
      double distance = 1000;
      double centerdistance = 1000;
      Vector3d tempPoint;
      NumericListKnot closestKnot = numericList.getLast ();
      Iterator<NumericListKnot> itr = numericList.iterator ();
      NumericListKnot minpointplanedistance = numericList.getLast ();
      NumericListKnot minpointplanedistance2 = numericList.getLast ();
      double threshold = 25; // need to be replaced with actual threshold

      /*
       * while (itr.hasNext ()) { tempPoint = new Point3d (itr.next().v); double
       * currentDistance = pointPlaneDistance(tempPoint, planeValues); if
       * (currentDistance < distance) { distance = currentDistance; closestKnot
       * = itr.next (); } }
       */
      
      //var for testing only
      double disttest = 1000;
      NumericListKnot knottest = numericList.getLast ();

      itr = numericList.iterator ();
      // Finding the point where tempPoint is closest to the plane
      while (itr.hasNext ()) {
         tempPoint = new Vector3d (itr.next ().v);
         double currentDistance =
            PlanePointDistance (worldNormal, worldPoint, tempPoint);
         double pointcenterDistance =
            getSquareDistance (new Point3d (tempPoint), worldPoint);
         if ((currentDistance < distance) && (itr.hasNext ())) {
            disttest = distance;
            knottest = minpointplanedistance;
            
            distance = currentDistance;
            minpointplanedistance = itr.next ();
         }
      }

      // itr = numericList.iterator ();
      // // Finding a second point that is closes to the plane and on the curve
      // // minimum threshold distance away from point minpointplanedistanceidx
      // distance = 1000;
      // while (itr.hasNext ()) {
      // tempPoint = new Vector3d (itr.next ().v);
      // double pointpointdis =
      // tempPoint.distance (new Vector3d (minpointplanedistance.v));
      // if ((pointpointdis > threshold) && (itr.hasNext ())) {
      // double pointplanedistance =
      // PlanePointDistance (worldNormal, worldPoint, tempPoint);
      // if ((pointplanedistance < distance) && (itr.hasNext ())) {
      // minpointplanedistance2 = itr.next ();
      // distance = pointplanedistance;
      // }
      // }
      // }

      closestKnot = minpointplanedistance;
      
      if(closestKnot.v.get (0) == 28.06773330132498) {
         System.out.println ("TRUEEE");
         closestKnot = knottest;
      }

      return closestKnot;
   }

   public NumericListKnot nextClosestNumericListKnotToPlaneMaxilla (
      Vector3d normal, Vector3d point, NumericList numericList,
      NumericListKnot prevClosest) {
      // Returns the closest knot in a numericList to a plane defined by a
      // normal and a point, and pose.

      // Transform normal and point so that they are in the correct orientation
      // and location in the world space
      Vector3d worldNormal = new Vector3d (normal);
      // worldNormal.transform (pose);
      // System.out.println (worldNormal);
      Point3d worldPoint = new Point3d (point);
      // worldPoint.transform (pose);

      // Getting plane equation from worldNormal and worldPoint
      double[] planeValues =
         planeEquationFromNormalAndPoint (worldNormal, worldPoint);

      // Setting up variables and iterator used in finding the point on the
      // curved line that is closest to the plane
      double distance = 1000;
      double centerdistance = 1000;
      Vector3d tempPoint;
      NumericListKnot closestKnot = numericList.getLast ();
      Iterator<NumericListKnot> itr = numericList.iterator ();
      NumericListKnot minpointplanedistance = numericList.getLast ();
      NumericListKnot minpointplanedistance2 = numericList.getLast ();
      double threshold = 10; // need to be replaced with actual threshold

      /*
       * while (itr.hasNext ()) { tempPoint = new Point3d (itr.next().v); double
       * currentDistance = pointPlaneDistance(tempPoint, planeValues); if
       * (currentDistance < distance) { distance = currentDistance; closestKnot
       * = itr.next (); } }
       */

      itr = numericList.iterator ();
      // Finding the point where tempPoint is closest to the plane
      while (itr.hasNext ()) {
         tempPoint = new Vector3d (itr.next ().v);
         double currentDistance =
            PlanePointDistance (worldNormal, worldPoint, tempPoint);
         if ((currentDistance < distance) && (itr.hasNext ())) {
            distance = currentDistance;
            minpointplanedistance = itr.next ();
         }
      }

      itr = numericList.iterator ();
      // Finding a second point that is closes to the plane and on the curve
      // minimum threshold distance away from point minpointplanedistance
      distance = 1000;
      while (itr.hasNext ()) {
         tempPoint = new Vector3d (itr.next ().v);
         double pointpointdis =
            tempPoint.distance (new Vector3d (minpointplanedistance.v));
         if ((pointpointdis > threshold) && (itr.hasNext ())) {
            double pointplanedistance =
               PlanePointDistance (worldNormal, worldPoint, tempPoint);
            if ((pointplanedistance < distance) && (itr.hasNext ())) {
               minpointplanedistance2 = itr.next ();
               distance = pointplanedistance;
            }

         }
      }
      // Checking which of the 2 points is not the prevClosest and set
      // closestKnot to that knot.
      closestKnot = new NumericListKnot (minpointplanedistance2);
      if (prevClosest.t == closestKnot.t) {
         closestKnot = new NumericListKnot (minpointplanedistance);
      }

      return closestKnot;
   }
   public AffineTransform3d calcLocCor (
      Vector3d x, Vector3d y, Vector3d z, Point3d t) {

      t.negate ();

      AffineTransform3d localCoord = new AffineTransform3d ();
      double[] row0 = { x.get (0), x.get (1), x.get (2), t.get (0) };
      double[] row1 = { y.get (0), y.get (1), y.get (2), t.get (1) };
      double[] row2 = { z.get (0), z.get (1), z.get (2), t.get (2) };
      double[] row3 = { 0.0, 0.0, 0.0, 1.0 };
      localCoord.setRow (0, row0);
      localCoord.setRow (1, row1);
      localCoord.setRow (2, row2);
      localCoord.setRow (3, row3);

      return localCoord;

   }
   public double calculateAngle (Vector3d normal1, Vector3d normal2) {
      double dot = normal1.dot (normal2);
      double norm = normal1.norm () * normal2.norm ();
      dot = dot / norm;
      return (Math.toDegrees (Math.acos (dot)));
   }

   public Point3d planelineintersection (
      Vector3d line, Vector3d normal, Vector3d start, Vector3d center) {
      Vector3d p0l0 = new Vector3d (center);
      p0l0.sub (start);

      double num = p0l0.dot (normal);
      double den = line.dot (normal);

      double d = num / den;
      // System.out.println("d:");
      // System.out.println(d);

      Point3d res = new Point3d (start);
      line.scale (d);
      res.add (line);

      return res;

   }

   public VectorNd projectPointToLine (Point3d point, Vector3d topPoint, Vector3d botPoint) {
      VectorNd projectedsurfaceside = new VectorNd ();
      VectorNd AP =
         new VectorNd (point)
            .sub (new VectorNd (topPoint.copy ()));
      VectorNd AB =
         new VectorNd (new VectorNd (botPoint.copy ()))
            .sub (new VectorNd (topPoint.copy ()));
      projectedsurfaceside =
         new VectorNd (new VectorNd (topPoint.copy ())).add (
            new VectorNd (AB).scale (
               new VectorNd (AP).dot (AB) / new VectorNd (AB).dot (AB)));

      
      return projectedsurfaceside;
   }
   
   public Point3d intersectLinePlane (Point3d point, Vector3d lineDir, Vector3d planePoint, Vector3d planeNormal) {
      if (Math.abs (lineDir.dot (planeNormal)) < 0.000001) {
         return new Point3d(0.0,0.0,0.0);
      }
      else {
         Vector3d w = new Point3d(point);
         w.sub (planePoint);
         Vector3d oppositeNormal = new Vector3d(planeNormal);
         oppositeNormal.negate ();
         double si = oppositeNormal.dot (w) / (lineDir.dot (planeNormal));
         Point3d intersectionPoint = new Point3d(w);
         Vector3d ext = new Vector3d (lineDir);
         ext.scale (si);
         intersectionPoint.add (ext);
         intersectionPoint.add (planePoint);
         return intersectionPoint;
      }
   }
   
   public Point2d intersectionLineLine (Point2d A, Point2d B, Point2d C, Point2d D) {     
          // Line AB represented as a1x + b1y = c1
          double a1 =  B.get (1) - A.get (1);
          double b1 = A.get (0) - B.get(0);          
          double c1 = a1*(A.get(0)) + b1*(A.get(1));
       
          // Line CD represented as a2x + b2y = c2
          double a2 = D.get(1) - C.get(1);
          double b2 = C.get(0) - D.get(0);
          double c2 = a2*(C.get(0))+ b2*(C.get(1));
       
          double determinant = a1*b2 - a2*b1;
       
          if (determinant == 0)
          {
              // The lines are parallel. 
              return null;
          }
          else
          {
              double x = (b2*c1 - b1*c2)/determinant;
              double y = (a1*c2 - a2*c1)/determinant;
              return new Point2d (x, y);
          }
      }
      
   public Vector3d ProjectPointToPlane (
      Vector3d src, Vector3d center, Vector3d normal) {
      Vector3d v = new Vector3d (src);
      v.sub (center);
      normal.normalize ();
      double d = v.dot (normal);
      Vector3d projected = new Vector3d (src);
      Vector3d scale = new Vector3d (normal);
      scale.scale (d);
      projected = projected.sub (scale);

      return projected;
   }

   public Vector3d ProjectLineToPlane (Vector3d line, Vector3d normal) {
      // line.normalize();
      normal.normalize ();

      double dot = line.dot (normal);
      // dot = dot / Math.pow(normal.norm(), 2);
      normal.scale (dot);
      line.sub (normal);

      return line.normalize ();
   }
   
   List<String> getRecordFromLine(String line) {
      List<String> values = new ArrayList<String>();
      try (Scanner rowScanner = new Scanner(line)) {
          rowScanner.useDelimiter(",");
          while (rowScanner.hasNext()) {
              values.add(rowScanner.next());
          }
      }
      return values;
}

   public int getIndexOfMax(double array[]) {
       if (array.length == 0) {
           return -1; // array contains no elements
       }
       double max = array[0];
       int pos = 0;
   
       for(int i=1; i<array.length; i++) {
           if (max < array[i]) {
               pos = i;
               max = array[i];
           }
       }
       return pos;
   }
   
   public int getIndexOfMax(int array[]) {
      if (array.length == 0) {
          return -1; // array contains no elements
      }
      int max = array[0];
      int pos = 0;
  
      for(int i=1; i<array.length; i++) {
          if (max < array[i]) {
              pos = i;
              max = array[i];
          }
      }
      return pos;
  }
   
   public int getIndexOfMin(int array[]) {
      if (array.length == 0) {
          return -1; // array contains no elements
      }
      int min = array[0];
      int pos = 0;
  
      for(int i=1; i<array.length; i++) {
          if (min > array[i]) {
              pos = i;
              min = array[i];
          }
      }
      return pos;
  }
   
   public int getIndexOfMin(double array[]) {
      if (array.length == 0) {
          return -1; // array contains no elements
      }
      double min = array[0];
      int pos = 0;
  
      for(int i=1; i<array.length; i++) {
          if (min > array[i]) {
              pos = i;
              min = array[i];
          }
      }
      return pos;
  }
   
   public LinkedList<Point3d> getLargestContour (ArrayList<LinkedList<Point3d>> contours){
      double[] perimeters = new double[contours.size ()];      
      for (int i=0; i <contours.size (); i++) {
         perimeters[i] = 0;
         for (int j=0; j<contours.get (i).size () - 1; j++) {
            perimeters[i] = perimeters[i] + getSquareDistance(contours.get (i).get (j), contours.get (i).get (j+1));
         }
      }
      return contours.get (getIndexOfMax(perimeters));      
   }
   
   public double get3DHandedHausdorff95 (PolygonalMesh A, PolygonalMesh B) {
      ArrayList<Vertex3d> verticesA = A.getVertices ();
      ArrayList<Vertex3d> verticesB = B.getVertices ();
      double[] h = new double[verticesA.size ()];
      
      for (int i = 0; i < verticesA.size (); i++) {
         Vertex3d vertA = verticesA.get (i);
         Point3d pA = vertA.getPosition ();
         
         double shortest = Double.POSITIVE_INFINITY;
         
         for (int j = 0; j < verticesB.size (); j++) {
            Vertex3d vertB = verticesB.get (j);
            Point3d pB = vertB.getPosition ();
            
            double currDist = Math.sqrt (getSquareDistance (pA, pB));
            if (currDist < shortest) {
               shortest = currDist;
            }
         }
         h[i] = shortest;
      }
      Arrays.sort (h);
      return h[(int)(0.95 * verticesA.size())];
   }
  
  public double get3DHausdorff95 (PolygonalMesh A, PolygonalMesh B) {
     double h1 = get3DHandedHausdorff95 (A, B);
     double h2 = get3DHandedHausdorff95 (B, A);
     
     if (h1 > h2) {
        return h1;
     } 
     else {
        return h2;
     }
     
  }
   
   public double getHandedHaussdorf (LinkedList<Point3d> points1, LinkedList<Point3d> points2) {
      double h = 0;
      for (int i = 0; i < points1.size (); i++ ) {
         double shortest = Double.POSITIVE_INFINITY;
         for (int j = 0; j<points2.size (); j++ ) {
            double dis = getSquareDistance(points1.get (i), points2.get (j));
            if (dis < shortest) {
               shortest = dis;
            }
        if (shortest > h) {
           h = shortest;
           }       
         }
      }
      return h;
   }  
      
   public double getHandedHaussdorf95 (LinkedList<Point3d> points1, LinkedList<Point3d> points2) {
      double[] h = new double[points1.size()];
      for (int i = 0; i < points1.size (); i++ ) {
         double shortest = Double.POSITIVE_INFINITY;
         for (int j = 0; j<points2.size (); j++ ) {
            double dis = getSquareDistance(points1.get (i), points2.get (j));
            if (dis < shortest) {
               shortest = dis;
            }
            h[i] = shortest;
         }
      }
      Arrays.sort (h);
      //returning 95th percentile
      return h[(int)(0.95 * points1.size())];
      
   }
   
   public double getHaussdorf (LinkedList<Point3d> points1, LinkedList<Point3d> points2) {
      double h1 = getHandedHaussdorf(points1, points2);
      double h2 = getHandedHaussdorf(points2, points1);
      if (h1 > h2) {
         return h1;
      }
      else {
         return h2;
      }
   }
   
   public double getHaussdorf95 (LinkedList<Point3d> points1, LinkedList<Point3d> points2) {
      double h1 = getHandedHaussdorf95(points1, points2);
      double h2 = getHandedHaussdorf95(points2, points1);
      if (h1 > h2) {
         return h1;
      }
      else {
         return h2;
      }
   }
   
   
   
}
