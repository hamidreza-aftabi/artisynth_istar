/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.istar.Prisman;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

// import org.apache.commons.math3.analysis.MultivariateFunction;
// import org.apache.commons.math3.exception.MathUnsupportedOperationException;
// import org.apache.commons.math3.optim.InitialGuess;
// import org.apache.commons.math3.optim.MaxEval;
// import org.apache.commons.math3.optim.PointValuePair;
// import org.apache.commons.math3.optim.SimpleBounds;
// import org.apache.commons.math3.optim.SimpleValueChecker;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.istar.Assist.Assist;
import maspack.collision.IntersectionContour;
import maspack.collision.IntersectionPoint;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.BVIntersector;
import maspack.geometry.Face;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.io.StlReader;
import maspack.interpolation.Interpolation;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.interpolation.Interpolation.Order;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.spatialmotion.ContactInfo;
import maspack.spatialmotion.SpatialInertia;

public class HelperMeshFunctions {
   public enum AlignmentType {
      RIGID, RIGID_WITH_SCALING, ORTHOGONAL, AFFINE
   }

   HelperMathFunctions mathHelper = new HelperMathFunctions ();

   public void setPlaneOrigin (PolygonalMesh plane, Vector3d origin) {
      Vector3d centroid = new Vector3d ();
      plane.computeCentroid (centroid);
      RigidTransform3d transMat = new RigidTransform3d ();
      transMat.setTranslation (new Vector3d (origin).sub (centroid));
      plane.transform (transMat);
   }

   public void createPlane (
      Vector3d sourceNormal, Vector3d targetNormal, Point3d targetOrigin,
      RigidTransform3d Pose, PolygonalMesh mesh) {

      //targetNormal.transform (Pose);
      //sourceNormal.transform (Pose);
      RotationMatrix3d rotmat0 = rotatePlane (sourceNormal, targetNormal);
      AffineTransform3d rotateplane0 = new AffineTransform3d ();
      rotateplane0.setRotation (rotmat0);
      mesh.transform (rotateplane0);
      setPlaneOrigin (mesh, targetOrigin);
   }

   public double ComputeContourArea (Vector3d widths) {
      // Compute area of contour based on 2 largest widths
      int maxID = 0;
      int minID = 0;
      // System.out.println (widths);

      double value = 10000;
      for (int i = 0; i < 3; i++) {
         if (widths.get (i) < value) {
            minID = i;
            value = widths.get (i);
         }
      }

      value = 0;
      for (int i = 0; i < 3; i++) {
         if (widths.get (i) > value) {
            maxID = i;
            value = widths.get (i);
         }
      }

      int midID = 3 - (maxID + minID);
      // System.out.println (maxID);
      // System.out.println (minID);
      // System.out.println (midID);

      double area = widths.get (maxID) * widths.get (midID);
      return area;
   }

   public Vector3d normalAtClosestVertex (PolygonalMesh model, Point3d point) {
      int numV = model.numVertices ();
      double mindistance = 1000000.0;
      double distance = 0.0;
      int idx = 0;

      for (int i = 0; i < numV; i++) {
         Vertex3d v = model.getVertex (i);
         distance = v.distance (point);
         if (distance < mindistance) {
            mindistance = distance;
            idx = i;
         }
      }
      Vertex3d v = model.getVertex (idx);
      Vector3d n = new Vector3d ();
      v.computeNormal (n);
      return n;
   }

   public Vector3d calculateAvgNormal (
      PolygonalMesh model, Vector3d point1, Vector3d point2) {
      Point3d point3 = null;
      Vector3d center =
         new Vector3d (
            (point1.get (0) + point2.get (0)) / 2,
            (point1.get (1) + point2.get (1)) / 2,
            (point1.get (2) + point2.get (2)) / 2);
      Point3d[] transformPoints = new Point3d[3];

      int numV = model.numVertices ();
      double mindistance = 1000000.0;
      // double mindistance = 100.0;
      double distance = 0.0;
      int idx1 = 0;
      int idx2 = 0;

      for (int i = 0; i < numV; i++) {
         Vertex3d v = model.getVertex (i);
         distance = v.distance (point1);
         if (distance < mindistance) {
            mindistance = distance;
            idx1 = i;
         }
      }

      mindistance = 1000000.0;
      for (int i = 0; i < numV; i++) {
         Vertex3d v = model.getVertex (i);
         distance = v.distance (point2);
         if (distance < mindistance) {
            mindistance = distance;
            idx2 = i;
         }
      }

      Vector3d normal1 = new Vector3d ();
      Vertex3d vertex1 = model.getVertex (idx1);
      vertex1.computeNormal (normal1);

      Vector3d normal2 = new Vector3d ();
      Vertex3d vertex2 = model.getVertex (idx2);
      vertex2.computeNormal (normal2);

      Vector3d averageNormal =
         new Vector3d (normal1).add (new Vector3d (normal2));
      averageNormal.scale (0.5);
      averageNormal.normalize ();

      /*
       * Vector3d directionBetweenPoints = new Vector3d();
       * directionBetweenPoints.sub (point1, point2);
       * directionBetweenPoints.normalize ();
       * 
       * double scalarProjection = averageNormal.dot(directionBetweenPoints);
       * 
       * Vector3d projAverageNormalonDirection = new Vector3d();
       * projAverageNormalonDirection = new
       * Vector3d(directionBetweenPoints).scale (scalarProjection);
       * 
       * Vector3d perpendicularNormal = new Vector3d();
       * 
       * perpendicularNormal.sub (averageNormal, projAverageNormalonDirection);
       * perpendicularNormal.normalize ();
       */
      return averageNormal;
   }

   public Point3d[] pointsForTransform (
      PolygonalMesh model, Vector3d point1, Vector3d point2) {
      Point3d point3 = null;
      Vector3d center =
         new Vector3d (
            (point1.get (0) + point2.get (0)) / 2,
            (point1.get (1) + point2.get (1)) / 2,
            (point1.get (2) + point2.get (2)) / 2);
      Point3d[] transformPoints = new Point3d[3];

      int numV = model.numVertices ();
      double mindistance = 1000000.0;
      // double mindistance = 100.0;
      double distance = 0.0;
      int idx1 = 0;
      int idx2 = 0;

      for (int i = 0; i < numV; i++) {
         Vertex3d v = model.getVertex (i);
         distance = v.distance (point1);
         if (distance < mindistance) {
            mindistance = distance;
            idx1 = i;
         }
      }

      mindistance = 1000000.0;
      for (int i = 0; i < numV; i++) {
         Vertex3d v = model.getVertex (i);
         distance = v.distance (point2);
         if (distance < mindistance) {
            mindistance = distance;
            idx2 = i;
         }
      }

      Vector3d normal1 = new Vector3d ();
      Vertex3d vertex1 = model.getVertex (idx1);
      vertex1.computeNormal (normal1);

      Vector3d normal2 = new Vector3d ();
      Vertex3d vertex2 = model.getVertex (idx2);
      vertex2.computeNormal (normal2);

      Vector3d averageNormal =
         new Vector3d (normal1).add (new Vector3d (normal2));
      averageNormal.normalize ();

      /*
       * Vector3d directionBetweenPoints = new Vector3d();
       * directionBetweenPoints.sub (point1, point2);
       * directionBetweenPoints.normalize ();
       * 
       * double scalarProjection = averageNormal.dot(directionBetweenPoints);
       * 
       * Vector3d projAverageNormalonDirection = new Vector3d();
       * projAverageNormalonDirection = new
       * Vector3d(directionBetweenPoints).scale (scalarProjection);
       * 
       * Vector3d perpendicularNormal = new Vector3d();
       * 
       * perpendicularNormal.sub (averageNormal, projAverageNormalonDirection);
       * perpendicularNormal.normalize ();
       */

      point3 = new Point3d (center.add (averageNormal.scale (0.01)));

      transformPoints[0] = new Point3d (point1);
      transformPoints[1] = new Point3d (point2);
      transformPoints[2] = point3;
      return transformPoints;
   }

   public Point3d[] pointsForTransform3 (
      Vector3d normal, Vector3d point1, Vector3d point2, Boolean extend) {
      Point3d point3 = null;
      Vector3d center =
         new Vector3d (
            (point1.get (0) + point2.get (0)) / 2,
            (point1.get (1) + point2.get (1)) / 2,
            (point1.get (2) + point2.get (2)) / 2);
      Point3d[] transformPoints = new Point3d[3];

      Vector3d averageNormal = new Vector3d (normal);
      averageNormal.normalize ();

      /*
       * Vector3d directionBetweenPoints = new Vector3d();
       * directionBetweenPoints.sub (point1, point2);
       * directionBetweenPoints.normalize ();
       * 
       * double scalarProjection = averageNormal.dot(directionBetweenPoints);
       * 
       * Vector3d projAverageNormalonDirection = new Vector3d();
       * projAverageNormalonDirection = new
       * Vector3d(directionBetweenPoints).scale (scalarProjection);
       * 
       * Vector3d perpendicularNormal = new Vector3d();
       * 
       * perpendicularNormal.sub (averageNormal, projAverageNormalonDirection);
       * perpendicularNormal.normalize ();
       */

      if (extend) {
         point3 = new Point3d (center.add (averageNormal.scale (0.01)));

         transformPoints[0] = new Point3d (point1);
         transformPoints[1] = new Point3d (point2);
         transformPoints[2] = point3;
         return transformPoints;
      }
      else {
         Vector3d extensionVector = new Vector3d (averageNormal);
         extensionVector.scale (100);
         // extensionVector.add (translationAdjustment);

         point3 = new Point3d (center.add (averageNormal.scale (0.01)));
         Point3d point1Copy = new Point3d (point1);
         point1Copy.add (extensionVector);
         Point3d point2Copy = new Point3d (point2);
         point2Copy.add (extensionVector);
         Point3d point3Copy = new Point3d (point3);
         point3Copy.add (extensionVector);

         // point3.add (new Vector3d(averageNormal).scale (100));

         transformPoints[0] = new Point3d (point1Copy);
         transformPoints[1] = new Point3d (point2Copy);
         transformPoints[2] = new Point3d (point3Copy);
         return transformPoints;
      }
   }

   public Point3d[] pointsForTransform2 (
      PolygonalMesh model, Vector3d point1, Vector3d point2) {
      Point3d point3 = null;
      Vector3d center =
         new Vector3d (
            (point1.get (0) + point2.get (0)) / 2,
            (point1.get (1) + point2.get (1)) / 2,
            (point1.get (2) + point2.get (2)) / 2);
      Point3d[] transformPoints = new Point3d[3];

      int numV = model.numVertices ();
      double mindistance = 1000000.0;
      // double mindistance = 100.0;
      double distance = 0.0;
      int idx1 = 0;
      int idx2 = 0;

      for (int i = 0; i < numV; i++) {
         Vertex3d v = model.getVertex (i);
         distance = v.distance (point1);
         if (distance < mindistance) {
            mindistance = distance;
            idx1 = i;
         }
      }

      mindistance = 1000000.0;
      for (int i = 0; i < numV; i++) {
         Vertex3d v = model.getVertex (i);
         distance = v.distance (point2);
         if (distance < mindistance) {
            mindistance = distance;
            idx2 = i;
         }
      }

      Vector3d normal1 = new Vector3d ();
      Vertex3d vertex1 = model.getVertex (idx1);
      vertex1.computeNormal (normal1);

      Vector3d normal2 = new Vector3d ();
      Vertex3d vertex2 = model.getVertex (idx2);
      vertex2.computeNormal (normal2);

      Vector3d translationAdjustment = point1.sub (point2);
      translationAdjustment.normalize ();
      // translationAdjustment.scale (200);

      // Vector3d averageNormal =
      // new Vector3d (normal1).add (new Vector3d (normal2));
      // averageNormal.normalize ();

      OBB targetBox = model.computeOBB ();
      Vector3d targetCenter = new Vector3d ();
      targetBox.getCenter (targetCenter);

      Vector3d targetCentroid = new Vector3d ();
      model.computeCentroid (targetCentroid);
      Vector3d targetFrontToBack = new Vector3d ();
      targetFrontToBack.sub (targetCentroid, targetCenter);
      targetFrontToBack.normalize ();

      int axisOfChoice = 1;
      Vector3d[] axes = getSortedAxes (model);

      double dot = targetFrontToBack.dot (axes[axisOfChoice]);
      double theta = Math.acos (dot);
      if (theta > Math.PI / 2) {
         axes[axisOfChoice].negate ();
      }

      dot = targetFrontToBack.dot (axes[0]);
      theta = Math.acos (dot);
      if (theta > Math.PI / 2) {
         axes[0].negate ();
      }

      Vector3d averageNormal = new Vector3d (axes[axisOfChoice]);
      averageNormal.normalize ();
      Vector3d axis0 = new Vector3d (axes[0].copy ()).normalize ();
      axis0.scale (0.5);
      averageNormal.add (axis0);
      averageNormal.normalize ();

      /*
       * Vector3d directionBetweenPoints = new Vector3d();
       * directionBetweenPoints.sub (point1, point2);
       * directionBetweenPoints.normalize ();
       * 
       * double scalarProjection = averageNormal.dot(directionBetweenPoints);
       * 
       * Vector3d projAverageNormalonDirection = new Vector3d();
       * projAverageNormalonDirection = new
       * Vector3d(directionBetweenPoints).scale (scalarProjection);
       * 
       * Vector3d perpendicularNormal = new Vector3d();
       * 
       * perpendicularNormal.sub (averageNormal, projAverageNormalonDirection);
       * perpendicularNormal.normalize ();
       */

      Vector3d extensionVector = new Vector3d (averageNormal);
      extensionVector.scale (300);
      // extensionVector.add (translationAdjustment);

      point3 = new Point3d (center.add (averageNormal.scale (0.01)));
      Point3d point1Copy = new Point3d (point1);
      point1Copy.add (extensionVector);
      Point3d point2Copy = new Point3d (point2);
      point2Copy.add (extensionVector);
      Point3d point3Copy = new Point3d (point3);
      point3Copy.add (extensionVector);

      // point3.add (new Vector3d(averageNormal).scale (100));

      transformPoints[0] = new Point3d (point1Copy);
      transformPoints[1] = new Point3d (point2Copy);
      transformPoints[2] = new Point3d (point3Copy);
      return transformPoints;
   }

   public Point3d[] fibulaPointsForTransformImplants (
      PolygonalMesh fibulaMesh, Vector3d fromCenterToSurface,
      Vector3d startPoint, Vector3d endPoint) {
      Point3d[] transformPoints = new Point3d[3];
      startPoint.add (fromCenterToSurface);
      endPoint.add (fromCenterToSurface);
      transformPoints = pointsForTransform (fibulaMesh, startPoint, endPoint);
      for (int i = 0; i < 3; i++) {
         transformPoints[i].sub (fromCenterToSurface);
      }
      return transformPoints;
   }

   public Point3d[] mandiblePointsForTransformImplants (
      NumericList plateNumericList, Vector3d startPoint, Vector3d endPoint) {
      int newSize = plateNumericList.getNumKnots ();
      Vector3d[] interpolatedPoints = new Vector3d[newSize];
      Iterator<NumericListKnot> itr = plateNumericList.iterator ();

      int i = 0;
      while (itr.hasNext ()) {
         interpolatedPoints[i] = new Point3d (itr.next ().v);
         i++;
      }
      Vector3d center =
         new Vector3d (
            (startPoint.get (0) + endPoint.get (0)) / 2,
            (startPoint.get (1) + endPoint.get (1)) / 2,
            (startPoint.get (2) + endPoint.get (2)) / 2);

      Vector3d[] normalVector = new Vector3d[3];
      Vector3d[] arrayOfPoints = new Vector3d[3];
      arrayOfPoints[0] = startPoint;
      arrayOfPoints[1] = endPoint;
      arrayOfPoints[2] = center;

      double d;
      for (int j = 0; j < 3; j++) {
         double distance = 9999;
         for (i = 0; i < interpolatedPoints.length; i++) {
            d = center.distance (interpolatedPoints[i]);
            if (d < distance) {
               normalVector[j] = new Vector3d ();
               normalVector[j].sub (arrayOfPoints[j], interpolatedPoints[i]);
               distance = d;
            }
         }
      }

      Vector3d averageNormal =
         new Vector3d (
            (normalVector[0].get (0) + normalVector[1].get (0)
            + normalVector[2].get (0)) / 3,
            (normalVector[0].get (1) + normalVector[1].get (1)
            + normalVector[2].get (1)) / 3,
            (normalVector[0].get (2) + normalVector[1].get (2)
            + normalVector[2].get (2)) / 3);

      Point3d point3 = new Point3d (center.add (averageNormal.scale (0.01)));

      Point3d[] transformPoints = new Point3d[3];
      transformPoints[0] = new Point3d (startPoint);
      transformPoints[1] = new Point3d (endPoint);
      transformPoints[2] = point3;
      return transformPoints;
   }

   public Vector3d averageVectorOut (
      PolygonalMesh model, Vector3d point1, Vector3d point2) {
      Point3d point3 = null;
      Vector3d center =
         new Vector3d (
            (point1.get (0) + point2.get (0)) / 2,
            (point1.get (1) + point2.get (1)) / 2,
            (point1.get (2) + point2.get (2)) / 2);
      Point3d[] transformPoints = new Point3d[3];

      int numV = model.numVertices ();
      double mindistance = 1000000.0;
      double distance = 0.0;
      int idx1 = 0;
      int idx2 = 0;

      for (int i = 0; i < numV; i++) {
         Vertex3d v = model.getVertex (i);
         distance = v.distance (point1);
         if (distance < mindistance) {
            mindistance = distance;
            idx1 = i;
         }
      }

      mindistance = 1000000.0;
      for (int i = 0; i < numV; i++) {
         Vertex3d v = model.getVertex (i);
         distance = v.distance (point2);
         if (distance < mindistance) {
            mindistance = distance;
            idx2 = i;
         }
      }

      Vector3d normal1 = new Vector3d ();
      Vertex3d vertex1 = model.getVertex (idx1);
      vertex1.computeNormal (normal1);

      Vector3d normal2 = new Vector3d ();
      Vertex3d vertex2 = model.getVertex (idx2);
      vertex2.computeNormal (normal2);

      Vector3d averageNormal =
         new Vector3d (normal1).add (new Vector3d (normal2));
      averageNormal.scale (0.5);
      averageNormal.normalize ();

      return averageNormal;

   }

   public Point3d[] pointsForTransform (
      Point3d point1, Point3d point2, Vector3d averageNormal) {
      Point3d point3 = null;
      Point3d[] transformPoints = new Point3d[3];
      Vector3d center =
         new Vector3d (
            (point1.get (0) + point2.get (0)) / 2,
            (point1.get (1) + point2.get (1)) / 2,
            (point1.get (2) + point2.get (2)) / 2);

      averageNormal.normalize ();

      point3 = new Point3d (center.add (averageNormal.scale (0.01)));

      transformPoints[0] = new Point3d (point1);
      transformPoints[1] = new Point3d (point2);
      transformPoints[2] = point3;

      return transformPoints;
   }

   public PolygonalMesh[] createSpheres (Vector3d[] points) {
      PolygonalMesh[] poly = new PolygonalMesh[3];
      poly[0] =
         MeshFactory
            .createSphere (
               1, 20, points[0].get (0), points[0].get (1), points[0].get (2));
      poly[1] =
         MeshFactory
            .createSphere (
               1, 20, points[1].get (0), points[1].get (1), points[1].get (2));
      poly[2] =
         MeshFactory
            .createSphere (
               1, 20, points[2].get (0), points[2].get (1), points[2].get (2));

      return poly;
   }

   public FixedMeshBody[] createSpheres2 (Vector3d[] points) {
      PolygonalMesh[] poly = new PolygonalMesh[3];
      FixedMeshBody[] bodies = new FixedMeshBody[3];
      poly[0] =
         MeshFactory
            .createSphere (
               1, 20, points[0].get (0), points[0].get (1), points[0].get (2));
      poly[1] =
         MeshFactory
            .createSphere (
               1, 20, points[1].get (0), points[1].get (1), points[1].get (2));
      poly[2] =
         MeshFactory
            .createSphere (
               1, 20, points[2].get (0), points[2].get (1), points[2].get (2));

      for (int i = 0; i < 3; i++) {
         bodies[i] = new FixedMeshBody ();
         bodies[i].setMesh (poly[i]);
      }
      return bodies;
   }

   public void visualizePoint (Vector3d point, MechModel model, String name) {
      PolygonalMesh tempMesh =
         MeshFactory
            .createSphere (1, 20, point.get (0), point.get (1), point.get (2));
      FixedMeshBody tempBody = new FixedMeshBody (name, tempMesh);
      model.addMeshBody (tempBody);
   }

   public void visualizeNumericList (
      NumericList numList, MechModel model, String name) {
      // Visualizing curatedList
      Iterator<NumericListKnot> curated_itr = numList.iterator ();
      int newSize = numList.getNumKnots ();
      Point3d[] interpolatedPoints = new Point3d[newSize];
      int[][] indicesTemp = new int[newSize - 1][2];

      int j = 0;
      while (curated_itr.hasNext ()) {
         interpolatedPoints[j] = new Point3d (curated_itr.next ().v);
         j++;
      }

      for (j = 0; j < newSize - 1; j++) {
         indicesTemp[j][0] = j;
         indicesTemp[j][1] = j + 1;
      }
      PolylineMesh sphericalPolyLineTemp =
         MeshFactory.createSphericalPolyline (50, 50, 50);

      PolylineMesh tempMesh = new PolylineMesh ();
      tempMesh.addMesh (sphericalPolyLineTemp);
      tempMesh.set (interpolatedPoints, indicesTemp);

      FixedMeshBody tempBody = new FixedMeshBody (name, tempMesh);
      model.addMeshBody (tempBody);
   }

   public PolygonalMesh readMesh (String path, String name) {
      PolygonalMesh mesh = null;
      try {
         // mesh = (PolygonalMesh)GenericMeshReader.readMesh(rbpath +
         // "MarzMandible.stl");
         System.out.println ("Opening: " + name);
         mesh = StlReader.read (path);
         System.out.println ("Opened: " + name);
      }
      catch (Exception e) {
         System.out.println ("Unable to read mesh: " + name);
         e.printStackTrace ();
         System.exit (1);
      }
      return mesh;
   }

   public RotationMatrix3d rotatePlane (
      Vector3d sourcenormal, Vector3d targetnormal) {
      double dot = sourcenormal.dot (targetnormal);
      double norm = sourcenormal.norm () * targetnormal.norm ();
      dot = dot / norm;
      Vector3d axis = new Vector3d ();
      axis.cross (sourcenormal, targetnormal);
      axis.normalize ();
      double c = dot;
      double s = Math.sqrt (1 - c * c);
      double CC = 1 - c;
      double x = axis.get (0);
      double y = axis.get (1);
      double z = axis.get (2);
      RotationMatrix3d rotate1 =
         new RotationMatrix3d (
            x * x * CC + c, x * y * CC - z * s, x * z * CC + y * s,
            y * x * CC + z * s, y * y * CC + c, y * z * CC - x * s,
            z * x * CC - y * s, z * y * CC + x * s, z * z * CC + c);
      return rotate1;
   }

   public RigidTransform3d SVDRegistration (
      Point3d[] target, Point3d[] source) {

      PolygonalMesh targetMesh = new PolygonalMesh ();
      PolygonalMesh sourceMesh = new PolygonalMesh ();

      // Creating Mesh from transform1
      Point3d p1 = target[0];
      Point3d p2 = target[1];
      Point3d p3 = target[2];
      double[] a1 = { 0.0, 0.0, 0.0 };
      p1.get (a1);
      /*
       * System.out.println(a1[0]); System.out.println(a1[1]);
       * System.out.println(a1[2]);
       */
      double[] a2 = { 0.0, 0.0, 0.0 };
      p2.get (a2);
      double[] a3 = { 0.0, 0.0, 0.0 };
      p3.get (a3);

      Vertex3d v1 = new Vertex3d (p1);
      Vertex3d v2 = new Vertex3d (p2);
      Vertex3d v3 = new Vertex3d (p3);

      targetMesh.addVertex (v1);
      targetMesh.addVertex (v2);
      targetMesh.addVertex (v3);
      // int index[] = {0, 1, 2};
      // targetMesh.addFace(index);
      targetMesh.addFace (v1, v2, v3);
      // mesh1.addFace(v1, v2, v3);

      // Creating Mesh from transform2
      Point3d p4 = source[0];
      Point3d p5 = source[1];
      Point3d p6 = source[2];
      double[] a4 = { 0.0, 0.0, 0.0 };
      p4.get (a4);
      double[] a5 = { 0.0, 0.0, 0.0 };
      p5.get (a5);
      double[] a6 = { 0.0, 0.0, 0.0 };
      p6.get (a6);

      Vertex3d v4 = new Vertex3d (p4);
      Vertex3d v5 = new Vertex3d (p5);
      Vertex3d v6 = new Vertex3d (p6);

      sourceMesh.addVertex (v4);
      sourceMesh.addVertex (v5);
      sourceMesh.addVertex (v6);
      // sourceMesh.addFace(index);
      sourceMesh.addFace (v4, v5, v6);
      // mesh2.addFace(v4, v5, v6);

      // Printing Points for Transform
      /*
       * System.out.println("Target Points:"); System.out.println(p1);
       * System.out.println(p2); System.out.println(p3);
       * System.out.println("Souce Points: "); System.out.println(p4);
       * System.out.println(p5); System.out.println(p6);
       */

      // Step 1: Compute Centroid
      Vector3d targetCentroid = new Vector3d ();
      targetMesh.computeCentroid (targetCentroid);
      /*
       * System.out.println("TargetCentroid: ");
       * System.out.println(targetCentroid);
       */
      Vector3d sourceCentroid = new Vector3d ();
      sourceMesh.computeCentroid (sourceCentroid);
      /*
       * System.out.println("SourceCentroid: ");
       * System.out.println(sourceCentroid);
       */

      // Step 2: Bringing both dataset to origin
      //
      double[] centeroftarget = { 0.0, 0.0, 0.0 };
      targetCentroid.get (centeroftarget);
      double[] centerofsource = { 0.0, 0.0, 0.0 };
      sourceCentroid.get (centerofsource);

      MatrixNd tilesource = new MatrixNd ();
      tilesource.setSize (3, 3);
      for (int i = 0; i < 3; i++) {
         tilesource.setRow (i, centerofsource);
      }

      MatrixNd tiletarget = new MatrixNd ();
      tiletarget.setSize (3, 3);
      for (int i = 0; i < 3; i++) {
         tiletarget.setRow (i, centeroftarget);
      }

      MatrixNd A = new MatrixNd ();
      MatrixNd B = new MatrixNd ();
      A.setSize (3, 3);
      B.setSize (3, 3);
      A.setRow (0, a4);
      A.setRow (1, a5);
      A.setRow (2, a6);
      B.setRow (0, a1);
      B.setRow (1, a2);
      B.setRow (2, a3);

      MatrixNd AA = new MatrixNd ();
      MatrixNd BB = new MatrixNd ();
      /*
       * System.out.println("A size"); System.out.println(A.colSize());
       * System.out.println(A.rowSize()); System.out.println("tilesource size");
       * System.out.println(tilesource.colSize());
       * System.out.println(tilesource.rowSize());
       */
      AA.sub (A, tilesource);
      BB.sub (B, tiletarget);

      RigidTransform3d transform = new RigidTransform3d ();
      /*
       * Vector3d translatetarget = new Vector3d(); Vector3d translatesource =
       * new Vector3d(); Vector3d origin = new Vector3d (0.0, 0.0, 0.0);
       * translatesource.sub(origin, sourceCentroid);
       * translatetarget.sub(origin, targetCentroid);
       */

      // Step 3: Finding Optimal Rotation (matrix R)
      //
      // Initializing Matrices
      RotationMatrix3d rotate = new RotationMatrix3d ();
      MatrixNd matH = new MatrixNd ();

      // Calculating H
      AA.transpose ();
      matH.mul (AA, BB);

      // SVD of H
      SVDecomposition SVD = null;
      try {
         SVD = new SVDecomposition (matH);
      }
      catch (Exception e) {
         System.out
            .println ("Error computing SVD of matH, size=" + matH.getSize ());
         System.out.println ("matH=\n" + matH);
         throw e;
      }
      MatrixNd V = SVD.getV ();
      MatrixNd U = SVD.getU ();
      MatrixNd Ut = new MatrixNd ();
      Ut.transpose (U);

      // Calculating rotation matrix R = V*Ut
      MatrixNd rot = new MatrixNd ();
      rot.mul (V, Ut);
      double determinant = rot.determinant ();
      /*
       * System.out.println("Determinant of Rot:");
       * System.out.println(determinant);
       */

      /*
       * System.out.println("Rot Matrix: "); System.out.println(rot);
       */

      // Handling Reflection Case
      if (determinant < 0) {
         // System.out.println ("Reflection Case Detected");
         double[] lastcol = { 0.0, 0.0, 0.0 };
         V.getColumn (2, lastcol);
         for (int i = 0; i < 3; i++) {
            lastcol[i] = lastcol[i] * (-1);
         }
         V.setColumn (2, lastcol);
         rot.mul (V, Ut);
         // System.out.println("New Determinant:");
         // System.out.println(rot.determinant());
      }

      double[] arrayval = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
      rot.get (arrayval);
      rotate.set (arrayval);
      transform.setRotation (rotate);
      /*
       * System.out.println("Rot Matrix: "); System.out.println(rot);
       * System.out.println("Rotate Matrix: "); System.out.println(rotate);
       */

      // Step 4: Calculate Translation Vector t
      rotate.negate (); // -R
      rotate.mul (sourceCentroid); // sourceCentroid = -R x sourceCentroid
      Vector3d translate = new Vector3d ();
      translate.add (sourceCentroid, targetCentroid); // -R x sourceCentroid +
                                                      // targetCentroid
      transform.setTranslation (translate);

      // Aligning mesh1 and mesh2
      // AffineTransform3d transform = MeshICP.align (targetMesh, sourceMesh,
      // MeshICP.AlignmentType.RIGID);

      /*
       * double dis1 = Math.sqrt(mathHelper.getSquareDistance (source[0],
       * source[1])); double dis2 = Math.sqrt (mathHelper.getSquareDistance
       * (target[0], target[1]));
       * 
       * System.out.print("dis1--"); System.out.println (dis1);
       * System.out.print("dis2--"); System.out.println (dis2);
       */

      sourceMesh.clear ();
      targetMesh.clear ();

      return transform;

   }

   public ArrayList<Point3d> CatmullRomChain (ArrayList<Vector3d> points) {
      int size = points.size ();
      ArrayList<Point3d> C = new ArrayList<Point3d> ();
      for (int i = 0; i < (size - 3); i++) {
         ArrayList<Point3d> c =
            CatmullRomSpline (
               points.get (i), points.get (i + 1), points.get (i + 2),
               points.get (i + 3));
         for (int j = 0; j < c.size (); j++) {
            C.add (c.get (j));
         }
      }
      return C;
   }

   public ArrayList<Point3d> CatmullRomSpline (
      Vector3d p1, Vector3d p2, Vector3d p3, Vector3d p4) {

      double t0 = 0.0;
      double t1 = calculateT (t0, p1, p2);
      double t2 = calculateT (t0, p2, p3);
      double t3 = calculateT (t0, p3, p4);
      ArrayList<Double> t = new ArrayList<Double> ();

      ArrayList<Point3d> c = new ArrayList<Point3d> ();
      for (int k = 0; k < 3; k++) {
         if (k == 0) {
            t = LinSpace (t0, t1, 20);
         }
         else if (k == 1) {
            t = LinSpace (t1, t2, 20);
         }
         else if (k == 2) {
            t = LinSpace (t2, t3, 20);
         }
         for (int i = 0; i < t.size (); i++) {
            double scale = (t1 - t.get (i)) / (t1 - t0);
            Point3d a = new Point3d (p1);
            a.scale (scale);
            scale = (t.get (i) - t0) / (t1 - t0);
            Point3d b = new Point3d (p2);
            b.scale (scale);
            Point3d A1 = new Point3d (a);
            A1.add (b);

            scale = (t2 - t.get (i)) / (t2 - t1);
            a = new Point3d (p2);
            a.scale (scale);
            scale = (t.get (i) - t1) / (t2 - t1);
            b = new Point3d (p3);
            b.scale (scale);
            Point3d A2 = new Point3d (a);
            A2.add (b);

            scale = (t3 - t.get (i)) / (t3 - t2);
            a = new Point3d (p2);
            a.scale (scale);
            scale = (t.get (i) - t2) / (t3 - t2);
            b = new Point3d (p3);
            b.scale (scale);
            Point3d A3 = new Point3d (a);
            A3.add (b);

            scale = (t2 - t.get (i)) / (t2 - t0);
            a = new Point3d (A1);
            a.scale (scale);
            scale = (t.get (i) - t0) / (t2 - t0);
            b = new Point3d (A2);
            b.scale (scale);
            Point3d B1 = new Point3d (a);
            B1.add (b);

            scale = (t3 - t.get (i)) / (t3 - t1);
            a = new Point3d (A2);
            a.scale (scale);
            scale = (t.get (i) - t1) / (t3 - t1);
            b = new Point3d (A3);
            b.scale (scale);
            Point3d B2 = new Point3d (a);
            B2.add (b);

            scale = (t2 - t.get (i)) / (t2 - t1);
            a = new Point3d (B1);
            a.scale (scale);
            scale = (t.get (i) - t1) / (t2 - t1);
            b = new Point3d (B2);
            b.scale (scale);
            Point3d C = new Point3d (a);
            C.add (b);

            c.add (C);
         }
      }
      return c;
   }

   public ArrayList<Double> LinSpace (double t1, double t2, int nPoints) {
      ArrayList<Double> d = new ArrayList<Double> ();
      double end = t2;
      double start = t1;
      if (t1 > t2) {
         end = t1;
         start = t2;
      }
      for (double i = start; i < end; i = (i + ((end - start) / nPoints))) {
         d.add (i);
      }
      return d;
   }

   public double calculateT (double t, Vector3d p1, Vector3d p2) {
      return Math.pow ((p1.distance (p2)), 0.5) + t;
   }

   public List<Integer> findIndexes (List<List<Double>> A, List<Double> f) {
      List<Integer> indexes = new ArrayList<Integer> ();

      int m = 0;
      for (int i = 0; i < A.size (); i++) {
         if (A.get (i) == f) {
            indexes.add (m, i);
            m++;
         }
      }

      return indexes;
   }

   public Boolean isMeshAbovePlane (PolygonalMesh plane, PolygonalMesh mesh) {
      int counter = 0;
      double sumResults = 0;
      for (Vertex3d vertex : mesh.getVertices ()) {
         int result = isVectorAbovePlane (plane, vertex.getWorldPoint ());
         counter += 1;
         sumResults += result;
         if (result == 0) {
            return Boolean.FALSE;
         }
      }
      // //if ((sumResults / counter) >= 0.99999){
      // if (sumResults < counter){
      // return Boolean.FALSE;
      // }
      // else {
      // return Boolean.TRUE;
      // }
      return Boolean.TRUE;
   }

   public Boolean isMeshBetweenPlanes (
      PolygonalMesh plane1, PolygonalMesh plane2, PolygonalMesh mesh) {
      int numPoints =
         plane1.getVertices ().size () + plane2.getVertices ().size ();
      Point3d[] points = new Point3d[numPoints];

      for (int i = 0; i < plane1.getVertices ().size (); i++) {
         points[i] = plane1.getVertices ().get (i).getWorldPoint ();
      }

      for (int j = plane1.getVertices ().size (); j < numPoints; j++) {
         points[j] =
            plane2
               .getVertices ().get (j - plane1.getVertices ().size ())
               .getWorldPoint ();
      }

      OBB box = new OBB ();
      box.set (points, 10, OBB.Method.ConvexHull);

      Vector3d meshCentroid = new Vector3d ();
      mesh.computeCentroid (meshCentroid);

      return box.containsPoint (new Point3d (meshCentroid));

   }

   public int isVectorAbovePlane (PolygonalMesh plane, Point3d vector) {
      Vector3d vectorCopy = new Vector3d (vector);
      Vector3d planeCentroid = new Vector3d ();
      Vector3d planeNormal = new Vector3d ();

      plane.computeCentroid (planeCentroid);
      planeNormal = plane.getFace (0).getNormal ();
      planeNormal.normalize ();

      double originalDis =
         mathHelper.PlanePointDistance (planeNormal, planeCentroid, vectorCopy);

      planeCentroid.add (planeNormal);

      double testDis =
         mathHelper.PlanePointDistance (planeNormal, planeCentroid, vectorCopy);

      if (testDis > originalDis) {
         // System.out.print ("Above");
         return 0;
      }
      else {
         // System.out.println ("Below");
         return 1;
      }

   }

   public int findIndex (List<List<Double>> A, List<Double> f) {
      int ind = -1;
      for (int i = 0; i < A.size (); i++) {
         if (A.get (i).equals (f)) {
            ind = i;
         }
      }
      return ind;
   }

   public void setMeshColour (PolygonalMesh mesh, float r, float g, float b) {
      // ///
      List<float[]> colourList = new ArrayList<float[]> ();
      for (int j = 0; j < mesh.numVertices (); j++) {
         float[] colour = { r, g, b };
         colourList.add (colour);
      }
      mesh.setColors (colourList, null);
   }

   public Vector3d[] getSortedAxes (PolygonalMesh mesh) {
      SpatialInertia inertia = new SpatialInertia ();
      mesh.computeInertia (inertia, 1.0);
      SymmetricMatrix3d mat = inertia.getRotationalInertia ();

      Vector3d[] sortedAxes = new Vector3d[3];
      Vector3d eig = new Vector3d ();
      Matrix3d V = new Matrix3d ();
      mat.getEigenValues_old (eig, V);

      double[] vecValues = new double[3];
      eig.get (vecValues);

      int minIndex = minIndex (vecValues);
      double min = vecValues[minIndex];

      for (int j = 0; j < 3; j++) {
         vecValues[j] = vecValues[j] / min;
         sortedAxes[j] = new Vector3d ();
      }

      int[] result = getIndicesInOrder (vecValues);

      // for(int j=0; j<3; j++) {
      // V.getRow (result[j], sortedAxes[j]);
      // }
      //
      // int[] result = getIndicesInOrder(vecValues);
      //
      // System.out.println(Arrays.toString(vecValues));
      // double[] array1 = { 1.1, 2.2, 3.3, 4.4, 3.3 };
      //
      // System.out.println(Arrays.toString(getIndicesInOrder(vecValues)));

      for (int j = 0; j < 3; j++) {
         V.getRow (result[j], sortedAxes[j]);
      }

      return sortedAxes;
   }

   public static int[] getIndicesInOrder (double[] array) {
      Map<Integer,Double> map = new HashMap<Integer,Double> (array.length);
      for (int i = 0; i < array.length; i++)
         map.put (i, array[i]);

      List<Entry<Integer,Double>> l =
         new ArrayList<Entry<Integer,Double>> (map.entrySet ());

      Collections.sort (l, new Comparator<Entry<?,Double>> () {
         @Override
         public int compare (Entry<?,Double> e1, Entry<?,Double> e2) {
            return e2.getValue ().compareTo (e1.getValue ());
         }
      });

      int[] result = new int[array.length];
      for (int i = 0; i < result.length; i++)
         result[i] = l.get (i).getKey ();

      return result;
   }

   public int minIndex (double... ds) {
      int idx = -1;
      double d = Double.POSITIVE_INFINITY;
      for (int i = 0; i < ds.length; i++)
         if (ds[i] < d) {
            d = ds[i];
            idx = i;
         }
      return idx;
   }
   
   public Point3d getCenterPoint (PolygonalMesh mesh) {
      ArrayList<Vertex3d> vertices = mesh.getVertices ();
      double count = 0.0;
      double xSum = 0.0;
      double ySum = 0.0;
      double zSum = 0.0;

      
      for (int i = 0; i < vertices.size (); i++) {
         Point3d point = vertices.get (i).getPosition ();
         xSum += point.x;
         ySum += point.y;
         zSum += point.z;
         count++;
      }
      
      double aveX = xSum / count;
      double aveY = ySum / count;
      double aveZ = zSum / count;
      
      Point3d point = new Point3d (aveX, aveY, aveZ);
      
      return point;
   }
  
   public double getVolumeOverlap(PolygonalMesh A, PolygonalMesh B) {
      if(A.equals (B)) {
         return 100.0;
      }
      
      double volume1 = A.computeVolume ();
      double volume2 = B.computeVolume ();
      
      PolygonalMesh intersectionMesh = new PolygonalMesh();
      
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector();
      intersectionMesh = intersector.findIntersection(A,B);
      if(intersectionMesh == null) {
         return 0.0;
      }
            
      double intersectionVol = intersectionMesh.computeVolume ();
      
      double simIndex = (2*intersectionVol) / (volume1 + volume2);
      
      return simIndex*100;
   }

   public Plane convertMeshToPlane (PolygonalMesh mesh) {

      Plane plane = new Plane ();
      Vector3d n1 = new Vector3d (mesh.getNormal (0)).normalize ();
      Vector3d cent1 = new Vector3d ();
      mesh.computeCentroid (cent1);
      Vector3d c1 = new Point3d (cent1);
      
      //source normal, target normal
      RotationMatrix3d r1 = rotatePlane (new Vector3d (0, 0, 1), new Vector3d (n1));
      RigidTransform3d t1 = new RigidTransform3d ();
      t1.setRotation (r1);
      t1.setTranslation (c1);
      plane.transform (t1);
      
      return plane;
   }
   
   public double getBonyContactScapula (PolygonalMesh mandible, PolygonalMesh segment, 
      List<PolygonalMesh> mandiblePlanes, MechModel mechModel) {
      
      double numerator = 0.0;
      double numContacts = 0.0;
      
      for (int i = 0; i < mandiblePlanes.size (); i++) {
         
         PolygonalMesh plane = mandiblePlanes.get (i);
         
         double x = getBonyContactMand (segment, mandible, plane, mechModel, i);
         numerator = numerator + x;
         numContacts++;
         
         System.out.println ("intersection " + i + " : " + x + "%");
      }
      
      double average = numerator / numContacts;
      
      return average;
   }
   
   public double getBonyContactMand (PolygonalMesh mandiblePiece, PolygonalMesh resection, PolygonalMesh segment, 
      PolygonalMesh plane, MechModel mechModel, int i, String s) {
      
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
      
      double areaMand = intersector.findIntersection (plane, resection).computeArea ();
      double areaSeg = intersector.findIntersection (plane, segment).computeArea ();
      
      PolygonalMesh temp = intersector.findIntersection (segment, resection);
//      RigidBody x = new RigidBody ("Overlap intersection " + i);
//      x.setSurfaceMesh (temp);
//      mechModel.add (x);
//      
//      RigidBody y = new RigidBody ("Mandible intersection " + i);
//      y.setSurfaceMesh (intersector.findIntersection (plane, resection));
//      mechModel.add (y);
      
      double totalArea = intersector.findIntersection (plane, temp).computeArea ();
      System.out.println ("Total Area " + i + ": " + totalArea);
      
      double simIndex = totalArea / areaMand;
      
      return simIndex;
   }
   
   public double getBonyContact (PolygonalMesh mandibleLeft, PolygonalMesh mandibleRight,
      PolygonalMesh mandibleFull, List<PolygonalMesh> donorSegmentMeshes, 
      List<PolygonalMesh> mandiblePlanes, MechModel mechModel) {
      
      double numerator = 0.0;
      double numContacts = 0.0;
      int segmentNum = 0;
      
      for (int i = 0; i < mandiblePlanes.size (); i++) {
         PolygonalMesh plane = mandiblePlanes.get (i);
         
         if (i == 0 || i == (mandiblePlanes.size () - 1)) {
            double x = 0.0;
            PolygonalMesh segment = new PolygonalMesh ();
            
            if (i == 0) {
               segment = donorSegmentMeshes.get (0);
               x = getBonyContactMand (segment, mandibleLeft, plane, mechModel, i);
            } else {
               segment = donorSegmentMeshes.get (donorSegmentMeshes.size() - 1);
               x = getBonyContactMand (segment, mandibleRight, plane, mechModel, i);
            }
            
            numerator = numerator + x;
            numContacts++;
            
            System.out.println ("mandible intersection " + i + " : " + x + "%");            
         } else {
            PolygonalMesh plane2 = mandiblePlanes.get (i + 1);
            
            PolygonalMesh segment1 = donorSegmentMeshes.get (segmentNum);
            PolygonalMesh segment2 = donorSegmentMeshes.get (segmentNum + 1);
            
            double x = getSegContact (segment1, segment2, plane, plane2, segmentNum, mechModel);

            numerator = numerator + x;
            numContacts++;
            i++;
            segmentNum++;
            
            System.out.println ("intersection between segment " + (segmentNum - 1) + " and segment " + 
               segmentNum + " : " + x + "%");
         }
         
      }
      double average = numerator / numContacts;
      
      return average;
   }
   
   public double getAreaFromContours (PolygonalMesh A, PolygonalMesh B) {
      SurfaceMeshIntersector intersect = new SurfaceMeshIntersector ();
      
      ArrayList<Point3d> contourPoints = new ArrayList<Point3d> ();
      ArrayList<IntersectionContour> x = intersect.findContours (A, B);
      for (int j = 0; j < x.size(); j++) {
         ArrayList<IntersectionPoint> m = x.get (0);
         for (int n = 0; n < m.size (); n++) {
            contourPoints.add (m.get (n));
         }
      }
      
      Vector3d xprodsum = new Vector3d (0,0,0);
      for (int j = 1; j < contourPoints.size () - 2; j++) {
         Vector3d vec1 = contourPoints.get (j).sub (contourPoints.get (0));
         Vector3d vec2 = contourPoints.get (j + 1).sub (contourPoints.get (0));
         xprodsum = xprodsum.add (vec1.cross (vec2));
      }
      
      double area = xprodsum.norm ();
      
      return area;
   }
   
   public double getBonyContactMand (PolygonalMesh segment, PolygonalMesh mandible,
      PolygonalMesh planarSurface, MechModel mechModel, int i) {
      
//      planarSurface.flip ();      
      SurfaceMeshIntersector intersect = new SurfaceMeshIntersector ();
      PolygonalMesh inter = intersect.findIntersection (mandible, segment);
//      
//      PolygonalMesh xxxx = intersect.findIntersection (mandible, planarSurface);
//      RigidBody xxx = new RigidBody ("ERROR MESH" + i);
//      xxx.setSurfaceMesh (xxxx);
//      mechModel.add (xxx);
//      
      double areaMand = getAreaFromContours (mandible, planarSurface);
      double areaSeg = getAreaFromContours (segment, planarSurface);
      double areaIntersect = getAreaFromContours (inter, planarSurface);
      
      System.out.println ("MAND: " + areaMand);
      System.out.println ("SEG: " + areaSeg);
      System.out.println ("INTERSECT: " + areaIntersect);
      
      double total = areaIntersect / areaMand;
      System.out.println ("ACTUAL: " + total);
      
      Plane plane = convertMeshToPlane (planarSurface);
      BVIntersector intersector = new BVIntersector ();
      
      ArrayList<LinkedList<Point3d>> contoursMand = new ArrayList<LinkedList<Point3d>> ();
      contoursMand = intersector.intersectMeshPlane (mandible, plane, 0.01);
      PolygonalMesh intersectMand = meshFromContour (contoursMand);
      
      ArrayList<LinkedList<Point3d>> contoursSeg = new ArrayList<LinkedList<Point3d>> ();
      contoursSeg = intersector.intersectMeshPlane (segment, plane, 0.01);
      PolygonalMesh intersectSeg = meshFromContour (contoursSeg);
      
      double area1 = intersectSeg.computeArea ();
      double area2 = intersectMand.computeArea ();
      
//      RigidBody x = new RigidBody ("mandible intersect " + i);
//      x.setSurfaceMesh (intersectMand);
//      mechModel.add (x);

      PolygonalMesh intermediate = MeshFactory.getIntersection (segment, mandible);
      ArrayList<LinkedList<Point3d>> contoursTotal = new ArrayList<LinkedList<Point3d>> ();
      contoursTotal = intersector.intersectMeshPlane (intermediate, plane, 0.01);
      PolygonalMesh totalIntersect = meshFromContour (contoursTotal);
      
      double intersectionArea = totalIntersect.computeArea ();
    
      double simIndex = 2 * (intersectionArea) / (area2 + area1);
      
      if (simIndex < 0) {
         return 0.0;
      }
    
      return simIndex * 100;
   }
   
   public double getAreaFromContour (ArrayList<IntersectionPoint> contour) {
      
      Vector3d xProdSum = new Vector3d (0,0,0);
      
      for (int i = 1; i < contour.size () - 1; i++) {
         Vector3d vec1 = contour.get (i).sub (contour.get (0));
         Vector3d vec2 = contour.get (i + 1).sub (contour.get (0));
         
         Vector3d crossed = vec1.cross (vec2);
         
         xProdSum = xProdSum.add (crossed);  
      }
      
      double area = xProdSum.norm ();
      
      return area;
   }
   
   public double getSegContact (PolygonalMesh seg1, PolygonalMesh seg2, 
      PolygonalMesh plane1, PolygonalMesh plane2, int segNum, MechModel mechModel) {  
   
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
      
      double area1 = intersector.findIntersection (plane1, seg1).computeArea ();
      double area2 = intersector.findIntersection (plane2, seg2).computeArea ();
      
      PolygonalMesh temp = intersector.findIntersection (seg1, seg2);
      
      double totalArea = intersector.findIntersection (temp, plane2).computeArea ();
   
      double simIndex = (2 * totalArea) / (area1 + area2);
 
      return simIndex * 100;
   }
   
   public PolygonalMesh meshFromContour (ArrayList<LinkedList<Point3d>> contours) {      
      
      PolygonalMesh intersect = new PolygonalMesh ();
            
      for (int i = 0; i < contours.size (); i++) {
         LinkedList<Point3d> pointList = contours.get (i);         
         for (int j = 0; j < pointList.size () - 1; j++) {
            intersect.addVertex (pointList.get (j));
         }
         
      }
      
      Vertex3d[] vertices = new Vertex3d[intersect.getVertices ().size ()];
      for (int i = 0; i < intersect.getVertices ().size (); i++) {
         vertices[i] = intersect.getVertices ().get (i);
      }
      
      intersect.addFace (vertices);
      
      return intersect;
   }
   
   public ArrayList<PolygonalMesh> getImplantList (MechModel mechModel){
      
      ArrayList<PolygonalMesh> implantList = new ArrayList<PolygonalMesh> ();
      Assist Assist = new Assist ();

      boolean x = true;
      for (int i = 0; x; i++) {
         if (Assist.GetMesh (mechModel, "implants" + i) != null) {
            PolygonalMesh imp = Assist.GetMesh (mechModel, "implants" + i).copy ();
            implantList.add (imp);
         } 
         else {
            x = false;
         }
      }
      
      return implantList;
   }
   
   public Vector3d[] getDentIntersections (ArrayList<PolygonalMesh> implantList, 
      Point3d[] interpolatedPoints) {
      
      Vector3d[] dentalImplantIntersections = new Vector3d[implantList.size ()];
      int i;
      
      for (i = 0; i < implantList.size (); i++) {
         OBB implantBox = implantList.get (i).computeOBB ();
         
         Vector3d[] implantDirs = new Vector3d[3];
         double distance = 1000;
         for (int j = 0; j < 3; j++) {
            implantDirs[j] = new Vector3d ();
         }
         
         implantBox.getSortedAxes (implantDirs);
         Vector3d implantCentroid = new Vector3d ();
         implantList.get (i).computeCentroid (implantCentroid);
         Vector3d x2 = new Vector3d ();
         Vector3d x1 = new Vector3d ();
         
         double implantHalfWidth = implantBox.getHalfWidths ().get (2);
         implantDirs[0].normalize ();
         implantDirs[0].scale (implantHalfWidth);
         x2.add (implantCentroid, implantDirs[0]);
         x1.sub (implantCentroid, implantDirs[0]);
         
         for (int j = 0; j < interpolatedPoints.length; j++) {
            Vector3d x1Minusx0 = new Vector3d ();
            Vector3d x2Minusx1 = new Vector3d ();

            x1Minusx0.sub (x1, interpolatedPoints[j]);
            x2Minusx1.sub (x2, x1);
            double t =
               -x1Minusx0.dot (x2Minusx1)
               / Math.pow (x2Minusx1.norm (), 2);

            Vector3d pointOnImplant = new Vector3d ();
            x2Minusx1.scale (t);
            pointOnImplant.add (x1, x2Minusx1);

            double d_a = pointOnImplant.distance (interpolatedPoints[j]);
            double d_b = 999999;
            double d = Math.min (d_a, d_b);

            if (d < distance) {
               dentalImplantIntersections[i] =
                  new Vector3d (pointOnImplant);
               distance = d;
            }
            
         }
      }

      return dentalImplantIntersections;
   }
   
   public Integer[] getRawOrderDental (ArrayList<PolygonalMesh> implantList, 
         Point3d[] interpolatedPoints) {
      
      Integer[] dentalImplantsRawOrder = new Integer[implantList.size ()];
      int i;
      
      for (i = 0; i < implantList.size (); i++) {
         OBB implantBox = implantList.get (i).computeOBB ();
         
         Vector3d[] implantDirs = new Vector3d[3];
         double distance = 1000;
         for (int j = 0; j < 3; j++) {
            implantDirs[j] = new Vector3d ();
         }
         
         implantBox.getSortedAxes (implantDirs);
         Vector3d implantCentroid = new Vector3d ();
         implantList.get (i).computeCentroid (implantCentroid);
         Vector3d x2 = new Vector3d ();
         Vector3d x1 = new Vector3d ();
         
         double implantHalfWidth = implantBox.getHalfWidths ().get (2);
         implantDirs[0].normalize ();
         implantDirs[0].scale (implantHalfWidth);
         x2.add (implantCentroid, implantDirs[0]);
         x1.sub (implantCentroid, implantDirs[0]);
         
         for (int j = 0; j < interpolatedPoints.length; j++) {
            Vector3d x1Minusx0 = new Vector3d ();
            Vector3d x2Minusx1 = new Vector3d ();

            x1Minusx0.sub (x1, interpolatedPoints[j]);
            x2Minusx1.sub (x2, x1);
            double t =
               -x1Minusx0.dot (x2Minusx1)
               / Math.pow (x2Minusx1.norm (), 2);

            Vector3d pointOnImplant = new Vector3d ();
            x2Minusx1.scale (t);
            pointOnImplant.add (x1, x2Minusx1);

            double d_a = pointOnImplant.distance (interpolatedPoints[j]);
            double d_b = 999999;
            double d = Math.min (d_a, d_b);

            if (d < distance) {
               dentalImplantsRawOrder[i] = j;
               distance = d;
            }
            

         }
         
         System.out.println ("Implant " + i + " interpolates at  "
            + dentalImplantsRawOrder[i]);

      }

      return dentalImplantsRawOrder;
   }
   
   public Integer[] orderImplantIndeces (Integer[] rawIndeces, 
         ArrayList<PolygonalMesh> implantList) {
      Integer[] orderedIndeces = new Integer[implantList.size ()];
    
      for (int i = 0; i < implantList.size (); i++) {
         int count = 0;
         for (int j = 0; j < implantList.size (); j++) {
            if (rawIndeces[j] < rawIndeces[i]) {
               count++;
            }
         } 
         orderedIndeces[count] = i;
      }
    
    
      return orderedIndeces;
   }
   
   public Vector3d[] orderImplantIntersections (Vector3d[] dentalImplantIntersections,
         Integer[] orderedIndeces, ArrayList<PolygonalMesh> implantList) {
      
      Vector3d[] orderedIntersections = new Vector3d[implantList.size ()];
      for (int i = 0; i < implantList.size (); i++) {
         orderedIntersections[i] =
            dentalImplantIntersections[orderedIndeces[i]];
         System.out.println (orderedIndeces[i]);
      }
      
      return orderedIntersections;
   }
   
   public NumericList getImplantNumeric (Vector3d[] orderedIntersects,
      VectorNd[] interpolatedVectors, int subDivisions, ArrayList<PolygonalMesh> implantList) {
      
      NumericList implantNumericList = new NumericList (3);
      Interpolation cubic = new Interpolation ();
      cubic.setOrder (Order.SphericalCubic);
      implantNumericList.setInterpolation (cubic);
      
      for (int i = 0; i < orderedIntersects.length; i++) {
         implantNumericList
            .add (
               orderedIntersects[i], i
               * subDivisions / (implantList.size () - 1));

      }
      interpolatedVectors = new VectorNd[subDivisions];
      
      for (int i = 0; i < subDivisions; i++) {
         interpolatedVectors[i] = new VectorNd ();
         interpolatedVectors[i].setSize (3);
         implantNumericList.interpolate (interpolatedVectors[i], i);

      }
      
      for (int i = 0; i < subDivisions; i++) {
         implantNumericList.add (interpolatedVectors[i], i);
      }
      
      return implantNumericList;
   }
   
}
