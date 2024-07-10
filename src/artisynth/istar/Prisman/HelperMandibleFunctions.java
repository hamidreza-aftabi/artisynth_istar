/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.istar.Prisman;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.ArtisynthPath;
import artisynth.istar.Assist.Assist;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;

public class HelperMandibleFunctions {

   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   RigidBody resectionMeshRigidBody = null;
   SurfaceMeshIntersector intersector =
      new maspack.collision.SurfaceMeshIntersector ();

   public List<Object> clipMandible2Planes (
      MechModel mechModel, PolygonalMesh mandibleMesh,
      PolygonalMesh resectionMesh, FixedMeshBody resectionMeshBody,
      PolygonalMesh nonResectionMesh1, PolygonalMesh nonResectionMesh2,
      Vector3d normalPlane1, Vector3d normalPlane2, Vector3d centerPlane1,
      Vector3d centerPlane2, PolygonalMesh plane1Mesh, PolygonalMesh plane2Mesh,
      FixedMeshBody nonResectionMeshBody, RigidBody mandibleMeshBody,
      RigidTransform3d posetr1, RigidTransform3d posetr2) {
      SurfaceMeshIntersector intersector =
         new maspack.collision.SurfaceMeshIntersector ();

      // Differencing Planes in sequence to get resection
      resectionMesh = intersector.findDifference01 (mandibleMesh, plane1Mesh);
      resectionMesh = intersector.findDifference01 (resectionMesh, plane2Mesh);
      resectionMeshBody = new FixedMeshBody ("Resection", resectionMesh);
      resectionMeshRigidBody = new RigidBody ("ResectionRigidBody");
      resectionMeshRigidBody.setSurfaceMesh (resectionMesh);

      // Creating new set of planes that have the opposite normals of the
      // clipping planes
      Vector3d mandibleCentroid = new Vector3d ();
      mandibleMesh.computeCentroid (mandibleCentroid);
      PolygonalMesh reversePlane1Mesh = MeshFactory.createPlane (90, 90);
      maspack.matrix.RigidTransform3d transform =
         new maspack.matrix.RigidTransform3d ();

      // MeshPlane is created at (0,0,0)
      transform.setTranslation (new Vector3d (mandibleCentroid));
      reversePlane1Mesh.transform (transform);

      // Rotating Plane to be upside down
      maspack.matrix.RigidTransform3d rotmat =
         new maspack.matrix.RigidTransform3d ();
      AxisAngle axisAngle = new AxisAngle ();
      axisAngle.set (new Vector3d (0, 1, 0), Math.PI);
      rotmat.setRotation (axisAngle);
      reversePlane1Mesh.transform (rotmat);

      // Creating second plane here as not not redo previous operations
      PolygonalMesh reversePlane2Mesh = reversePlane1Mesh.copy ();

      // Creating planes that have opposite normals from the resection plane
      // based on the loaded file
      Vector3d currentPlane1Normal =
         new Vector3d (reversePlane1Mesh.getNormal (0));
      Vector3d currentPlane2Normal =
         new Vector3d (reversePlane2Mesh.getNormal (0));
      // RigidTransform3d pose1 = plane1MeshBody.getPose ();
      // RigidTransform3d pose2 = plane2MeshBody.getPose ();
      // currentPlane1Normal.transform (pose1);
      // currentPlane2Normal.transform (pose2);

      RotationMatrix3d rotatePlane1 =
         meshHelper
            .rotatePlane (
               currentPlane1Normal, new Vector3d (normalPlane1).negate ());
      RotationMatrix3d rotatePlane2 =
         meshHelper
            .rotatePlane (
               currentPlane2Normal, new Vector3d (normalPlane2).negate ());

      AffineTransform3d t1 = new AffineTransform3d ();
      AffineTransform3d t2 = new AffineTransform3d ();
      t1.setRotation (rotatePlane1);
      t2.setRotation (rotatePlane2);

      reversePlane1Mesh.transform (t1);
      reversePlane2Mesh.transform (t2);

      meshHelper.setPlaneOrigin (reversePlane1Mesh, centerPlane1);
      meshHelper.setPlaneOrigin (reversePlane2Mesh, centerPlane2);

      // Corrective Transform of Plane1
      Vector3d originalCentroidPlane1 = new Vector3d ();
      plane1Mesh.computeCentroid (originalCentroidPlane1);

      Vector3d newCentroid1 = new Vector3d ();
      reversePlane1Mesh.computeCentroid (newCentroid1);
      RigidTransform3d correctiveTranslationPlane1 = new RigidTransform3d ();
      correctiveTranslationPlane1
         .setTranslation (
            new Vector3d (originalCentroidPlane1).sub (newCentroid1));
      reversePlane1Mesh.transform (correctiveTranslationPlane1);
      reversePlane1Mesh.setMeshToWorld (plane1Mesh.getMeshToWorld ());
      plane1Mesh.getMeshToWorld (posetr1);

      // Corrective Transform of Plane2
      Vector3d originalCentroidPlane2 = new Vector3d ();
      plane2Mesh.computeCentroid (originalCentroidPlane2);
      plane2Mesh.getMeshToWorld (posetr2);

      Vector3d newCentroid2 = new Vector3d ();
      reversePlane2Mesh.computeCentroid (newCentroid2);
      RigidTransform3d correctiveTranslationPlane2 = new RigidTransform3d ();
      correctiveTranslationPlane2
         .setTranslation (
            new Vector3d (originalCentroidPlane2).sub (newCentroid2));
      reversePlane2Mesh.transform (correctiveTranslationPlane2);
      reversePlane2Mesh.setMeshToWorld (plane2Mesh.getMeshToWorld ());

      // Differencing reverse plane from first mesh
      intersector = new maspack.collision.SurfaceMeshIntersector ();

      nonResectionMesh1 =
         intersector.findDifference01 (mandibleMesh, reversePlane1Mesh);

      // Differencing reverse plane from second mesh
      nonResectionMesh2 =
         intersector.findDifference01 (mandibleMesh, reversePlane2Mesh);

      // Combining first and second unresected mesh
      PolygonalMesh nonResectionMesh = new PolygonalMesh (nonResectionMesh1);
      nonResectionMesh.addMesh (nonResectionMesh1);

      // Commenting out for marie's case
      nonResectionMesh =
         MeshFactory.getUnion (nonResectionMesh, nonResectionMesh2);

      nonResectionMeshBody = new FixedMeshBody ("Unresected", nonResectionMesh);

      // Finding which non-resection mesh is the side that is plated first
      Point3d centroid1 = new Point3d ();
      nonResectionMesh1.computeCentroid (centroid1);
      Point3d centroid2 = new Point3d ();
      nonResectionMesh2.computeCentroid (centroid2);

      Point3d firstMarker = new Point3d ();
      FrameMarker[] LineFrameMarkers = mandibleMeshBody.getFrameMarkers ();

      if (LineFrameMarkers.length != 0) {
         firstMarker = LineFrameMarkers[0].getPosition ();

         double distance1 = centroid1.distance (firstMarker);
         double distance2 = centroid2.distance (firstMarker);

         if (distance2 < distance1) {
            PolygonalMesh temp = new PolygonalMesh (nonResectionMesh1);
            nonResectionMesh1 = new PolygonalMesh (nonResectionMesh2);
            nonResectionMesh2 = new PolygonalMesh (temp);
         }
      }

      List<Object> returnList = new ArrayList<Object> ();
      returnList.add (resectionMesh);
      returnList.add (resectionMeshBody);
      returnList.add (nonResectionMesh1);
      returnList.add (nonResectionMesh2);
      returnList.add (nonResectionMeshBody);
      returnList.add (reversePlane1Mesh);
      returnList.add (reversePlane2Mesh);

      return returnList;
   }

   public List<Object> clipMandible1Plane (
      MechModel mechModel, PolygonalMesh mandibleMesh,
      PolygonalMesh resectionMesh, FixedMeshBody resectionMeshBody,
      PolygonalMesh nonResectionMesh1, Vector3d normalPlane1,
      Vector3d centerPlane1, PolygonalMesh plane1Mesh,
      FixedMeshBody nonResectionMeshBody, RigidBody mandibleMeshBody,
      File setPlaneFile, RigidTransform3d posetr1) {
      SurfaceMeshIntersector intersector =
         new maspack.collision.SurfaceMeshIntersector ();

      // Differencing Planes in sequence to get resection
      resectionMesh = intersector.findDifference01 (mandibleMesh, plane1Mesh);
      resectionMeshBody = new FixedMeshBody ("Resection", resectionMesh);
      resectionMeshRigidBody = new RigidBody ("ResectionRigidBody");
      resectionMeshRigidBody.setSurfaceMesh (resectionMesh);

      // Creating new set of planes that have the opposite normals of the
      // clipping planes
      Vector3d mandibleCentroid = new Vector3d ();
      mandibleMesh.computeCentroid (mandibleCentroid);
      PolygonalMesh reversePlane1Mesh = MeshFactory.createPlane (90, 90);
      maspack.matrix.RigidTransform3d transform =
         new maspack.matrix.RigidTransform3d ();

      // MeshPlane is created at (0,0,0)
      transform.setTranslation (new Vector3d (mandibleCentroid));
      reversePlane1Mesh.transform (transform);

      // Rotating Plane to be upside down
      maspack.matrix.RigidTransform3d rotmat =
         new maspack.matrix.RigidTransform3d ();
      AxisAngle axisAngle = new AxisAngle ();
      axisAngle.set (new Vector3d (0, 1, 0), Math.PI);
      rotmat.setRotation (axisAngle);
      reversePlane1Mesh.transform (rotmat);

      // Creating planes that have opposite normals from the resection plane
      // based on the loaded file
      Vector3d currentPlane1Normal =
         new Vector3d (reversePlane1Mesh.getNormal (0));

      RotationMatrix3d rotatePlane1 =
         meshHelper
            .rotatePlane (
               currentPlane1Normal, new Vector3d (normalPlane1).negate ());

      AffineTransform3d t1 = new AffineTransform3d ();
      t1.setRotation (rotatePlane1);

      reversePlane1Mesh.transform (t1);

      meshHelper.setPlaneOrigin (reversePlane1Mesh, centerPlane1);

      // Corrective Transform of Plane1
      Vector3d originalCentroidPlane1 = new Vector3d ();
      plane1Mesh.computeCentroid (originalCentroidPlane1);

      Vector3d newCentroid1 = new Vector3d ();
      reversePlane1Mesh.computeCentroid (newCentroid1);
      RigidTransform3d correctiveTranslationPlane1 = new RigidTransform3d ();
      correctiveTranslationPlane1
         .setTranslation (
            new Vector3d (originalCentroidPlane1).sub (newCentroid1));
      reversePlane1Mesh.transform (correctiveTranslationPlane1);
      reversePlane1Mesh.setMeshToWorld (plane1Mesh.getMeshToWorld ());
      plane1Mesh.getMeshToWorld (posetr1);

      // Differencing reverse plane from first mesh
      intersector = new maspack.collision.SurfaceMeshIntersector ();

      nonResectionMesh1 =
         intersector.findDifference01 (mandibleMesh, reversePlane1Mesh);

      // Combining first and second unresected mesh
      PolygonalMesh nonResectionMesh = new PolygonalMesh (nonResectionMesh1);
      nonResectionMesh.addMesh (nonResectionMesh1);

      nonResectionMeshBody = new FixedMeshBody ("Unresected", nonResectionMesh);

      // Finding which non-resection mesh is the side that is plated first
      Point3d centroid1 = new Point3d ();
      nonResectionMesh1.computeCentroid (centroid1);

      List<Object> returnList = new ArrayList<Object> ();
      returnList.add (resectionMesh);
      returnList.add (resectionMeshBody);
      returnList.add (nonResectionMesh1);
      returnList.add (nonResectionMeshBody);
      returnList.add (reversePlane1Mesh);

      return returnList;
   }

   public List<Object> prepareDonor (
      Point3d disPoint, Float DonorDistanceProx, Float DonorDistanceDis,
      PolygonalMesh DonorCuttingPlanes[], PolygonalMesh clippedDonorMesh,
      PolygonalMesh DonorMesh, PolygonalMesh scapulaTrimPlane,
      Boolean DonorOption, FixedMeshBody DonorMeshBody,
      Boolean automaticScapulaTransform) {

      RigidTransform3d poseDonor = DonorMeshBody.getPose ();
      Point3d fibcen = new Point3d ();
      DonorMesh.computeCentroid (fibcen);
      fibcen.transform (poseDonor);
      // System.out.println("Donor Center:");
      // System.out.println(fibcen);
      OBB boundingbox = DonorMesh.computeOBB ();
      PolygonalMesh trimmedScapula = null;
      RigidBody trimmedScapulaBody;

      if (DonorOption) {
         trimmedScapula = new PolygonalMesh (DonorMesh.clone ());
         trimmedScapula =
            intersector.findDifference01 (trimmedScapula, scapulaTrimPlane);
         trimmedScapulaBody = new RigidBody ("Trimmed Scapula");
         trimmedScapulaBody.setSurfaceMesh (trimmedScapula);

         poseDonor = trimmedScapulaBody.getPose ();
         trimmedScapula.computeCentroid (fibcen);
         fibcen.transform (poseDonor);
         boundingbox = trimmedScapula.computeOBB ();
      }

      RigidTransform3d posebb = new RigidTransform3d ();
      boundingbox.getTransform (posebb);
      Vector3d widths = new Vector3d ();
      boundingbox.getHalfWidths (widths);
      // widths.transform(posebb);
      // System.out.println("BoundingBox Width:");
      // System.out.println(widths);
      Point3d bbcenter = new Point3d ();
      boundingbox.getCenter (bbcenter);
      bbcenter.transform (poseDonor);
      // System.out.println("BoundingBox Center:");
      // System.out.println(bbcenter);
      Vector3d[] axes = new Vector3d[3];
      for (int i = 0; i < axes.length; i++) {
         axes[i] = new Vector3d ();
      }
      boundingbox.getSortedAxes (axes);
      // System.out.println("BoundingBox Axes:");
      for (int i = 0; i < 3; i++) {
         axes[i].transform (poseDonor);
         // System.out.println(axes[i]);
      }
      Vector3d vec = new Vector3d (axes[0].copy ().negate ());
      // System.out.println("boundbox dir: ");
      // System.out.println(vec);

      double width = 0.0;
      for (int i = 0; i < 3; i++) {
         if (widths.get (i) > width) {
            width = widths.get (i);
         }
      }
      double x = axes[0].get (0);
      double y = axes[0].get (1);
      double z = axes[0].get (2);
      x = x * width + bbcenter.get (0);
      y = y * width + bbcenter.get (1);
      z = z * width + bbcenter.get (2);
      Point3d proxPoint = new Point3d (x, y, z);
      x = axes[0].get (0) * (-1);
      y = axes[0].get (1) * (-1);
      z = axes[0].get (2) * (-1);
      x = x * width + bbcenter.get (0);
      y = y * width + bbcenter.get (1);
      z = z * width + bbcenter.get (2);
      disPoint = new Point3d (x, y, z);
      /*
       * System.out.println("New ProxPoint:"); System.out.println(proxPoint);
       * System.out.println("New DisPoint:"); System.out.println(disPoint);
       */

      Vector3d dir = new Vector3d (vec);
      dir.normalize ();
      Vector3d proxTranslationVector = new Vector3d (dir.copy ());
      Vector3d disTranslationVector = new Vector3d (dir.copy ());

      // Working on plane closest to proximal end
      RigidTransform3d proxTransform = new RigidTransform3d ();
      // Scale translation Vector by value inputed by user
      proxTranslationVector.scale (DonorDistanceProx);
      proxTransform.setTranslation (proxTranslationVector);
      proxPoint.transform (proxTransform);
      DonorCuttingPlanes[0] = MeshFactory.createPlane (90, 90, 30, 30);
      FixedMeshBody Donorclipping1 =
         new FixedMeshBody ("Donor Cutting Plane 0", DonorCuttingPlanes[0]);
      RigidTransform3d poseclipping0 = Donorclipping1.getPose ();
      Vector3d normal1 = DonorCuttingPlanes[0].getNormal (0);
      meshHelper
         .createPlane (
            normal1, dir, proxPoint, poseclipping0, DonorCuttingPlanes[0]);

      // Working on plane closest to distal end
      RigidTransform3d disTransform = new RigidTransform3d ();
      // Reverses transaltion vector
      disTranslationVector.scale (-DonorDistanceDis);
      disTransform.setTranslation (disTranslationVector);
      disPoint.transform (disTransform);
      // Reverse dir
      dir.negate ();
      DonorCuttingPlanes[1] = MeshFactory.createPlane (90, 90, 30, 30);
      FixedMeshBody Donorclipping2 =
         new FixedMeshBody ("Donor Cutting Plane 1", DonorCuttingPlanes[1]);
      RigidTransform3d poseclipping2 = Donorclipping2.getPose ();
      Vector3d normal2 = DonorCuttingPlanes[1].getNormal (0);
      meshHelper
         .createPlane (
            normal2, dir, disPoint, poseclipping2, DonorCuttingPlanes[1]);

      if (!DonorOption) {
         clippedDonorMesh =
            intersector.findDifference01 (DonorMesh, DonorCuttingPlanes[0]);
         clippedDonorMesh =
            intersector
               .findDifference01 (clippedDonorMesh, DonorCuttingPlanes[1]);
      }
      else {
         if (automaticScapulaTransform) {
            clippedDonorMesh =
               intersector
                  .findDifference01 (trimmedScapula, DonorCuttingPlanes[0]);
            clippedDonorMesh =
               intersector
                  .findDifference01 (clippedDonorMesh, DonorCuttingPlanes[1]);
         }
         else {
            clippedDonorMesh = new PolygonalMesh (trimmedScapula.clone ());
         }
      }

      List<Object> returnList = new ArrayList<Object> ();
      returnList.add (disPoint);
      returnList.add (DonorCuttingPlanes);
      returnList.add (clippedDonorMesh);

      return returnList;
   }

   public PolygonalMesh visualizeFibulaCuts (
      MechModel mechModel, FixedMeshBody resectionFibulaMeshBody,
      FixedMeshBody nonResectionFibulaMeshBody,
      PolygonalMesh[] nonResectionFibulaMeshes,
      PolygonalMesh[] resectionFibulaMeshes, List<PolygonalMesh> cuttingPlanes,
      PolygonalMesh clippedDonorMesh,
      List<FixedMeshBody> cuttingPlanesMeshBodies,
      PolygonalMesh nonResectionFibulaMesh, RigidBody clippedDonorMeshBody) {

      nonResectionFibulaMeshes = new PolygonalMesh[cuttingPlanes.size () / 2];
      resectionFibulaMeshes = new PolygonalMesh[cuttingPlanes.size () / 2];

      mechModel.removeMeshBody (resectionFibulaMeshBody);
      mechModel.removeMeshBody (nonResectionFibulaMeshBody);
      for (int i = 0; i < cuttingPlanes.size () / 2; i++) {
         nonResectionFibulaMeshes[i] = new PolygonalMesh ();
         resectionFibulaMeshes[i] = new PolygonalMesh ();
      }

      for (int i = 0; i < cuttingPlanes.size () - 1; i = i + 2) {
         PolygonalMesh firstPlane = cuttingPlanes.get (i);
         PolygonalMesh secondPlane = cuttingPlanes.get (i + 1);
         FixedMeshBody firstPlaneBody = cuttingPlanesMeshBodies.get (i);
         FixedMeshBody secondPlaneBody = cuttingPlanesMeshBodies.get (i + 1);

         Vector3d translateOrig1 = new Vector3d ();
         Vector3d translateOrig2 = new Vector3d ();
         firstPlane.computeCentroid (translateOrig1);
         secondPlane.computeCentroid (translateOrig2);

         correctiveTransformPlane (firstPlane, firstPlaneBody);
         correctiveTransformPlane (secondPlane, secondPlaneBody);

         SurfaceMeshIntersector intersector =
            new maspack.collision.SurfaceMeshIntersector ();

         // Differencing Planes in sequence to get resection
         resectionFibulaMeshes[i / 2] =
            intersector.findDifference01 (clippedDonorMesh, firstPlane);
         resectionFibulaMeshes[i / 2] =
            intersector
               .findDifference01 (resectionFibulaMeshes[i / 2], secondPlane);

         // Create planes whose normals are opposite of cutting planes
         PolygonalMesh reversePlane1Mesh = MeshFactory.createPlane (90, 90);

         Point3d centroidPlane1 = new Point3d ();
         centroidPlane1.x = translateOrig1.x;
         centroidPlane1.y = translateOrig1.y;
         centroidPlane1.z = translateOrig1.z;
         meshHelper
            .createPlane (
               reversePlane1Mesh.getNormal (0),
               firstPlane.getNormal (0).negate (), centroidPlane1,
               firstPlaneBody.getPose (), reversePlane1Mesh);

         PolygonalMesh reversePlane2Mesh = MeshFactory.createPlane (90, 90);

         Point3d centroidPlane2 = new Point3d ();
         centroidPlane2.x = translateOrig2.x;
         centroidPlane2.y = translateOrig2.y;
         centroidPlane2.z = translateOrig2.z;
         meshHelper
            .createPlane (
               reversePlane2Mesh.getNormal (0),
               secondPlane.getNormal (0).negate (), centroidPlane2,
               secondPlaneBody.getPose (), reversePlane2Mesh);

         // Differencing reverse plane from first mesh
         intersector = new maspack.collision.SurfaceMeshIntersector ();

         nonResectionFibulaMeshes[i / 2] =
            intersector.findDifference01 (clippedDonorMesh, reversePlane1Mesh);

         // Differencing reverse plane from second mesh
         clippedDonorMesh =
            intersector.findDifference01 (clippedDonorMesh, reversePlane2Mesh);
      }

      // Combining all nonresected fibula mesh segments
      nonResectionFibulaMesh = new PolygonalMesh (clippedDonorMesh);
      nonResectionFibulaMesh.addMesh (clippedDonorMesh);
      for (int j = 0; j < nonResectionFibulaMeshes.length; j++) {
         nonResectionFibulaMesh =
            MeshFactory
               .getUnion (nonResectionFibulaMesh, nonResectionFibulaMeshes[j]);
      }
      nonResectionFibulaMeshBody =
         new FixedMeshBody ("Unresected Fibula", nonResectionFibulaMesh);
      // mechModel.addMeshBody (nonResectionFibulaMeshBody);

      // Combining all resected fibula mesh segments
      PolygonalMesh resectionFibulaMesh =
         new PolygonalMesh (resectionFibulaMeshes[0]);
      resectionFibulaMesh.addMesh (resectionFibulaMeshes[0]);
      for (int j = 1; j < resectionFibulaMeshes.length; j++) {
         resectionFibulaMesh =
            MeshFactory
               .getUnion (resectionFibulaMesh, resectionFibulaMeshes[j]);
      }
      resectionFibulaMeshBody =
         new FixedMeshBody ("Resected Fibula", resectionFibulaMesh);
      // mechModel.addMeshBody (resectionFibulaMeshBody);

      clippedDonorMeshBody.getRenderProps ().setVisible (false);
      return nonResectionFibulaMesh;
   }

   public List<FixedMeshBody> ManualSplit (
      MechModel mechModel, RigidBody Plane, RigidBody Donor, Integer count) {
      SurfaceMeshIntersector intersector =
         new maspack.collision.SurfaceMeshIntersector ();
      PolygonalMesh segmentPlane = Plane.getSurfaceMesh ().clone ();
//      String homedir = ArtisynthPath.getHomeDir ();
//      File pathHome = new File (homedir);
//      String homeParent = pathHome.getParentFile ().getAbsolutePath ();
//      String path =
//         homeParent
//         + "/artisynth_projects/src/artisynth/models/Prisman/stl/horizontal.stl";
      Assist Assist = new Assist();
//      PolygonalMesh metalinsertlong = Assist.loadGeometry(path);
      segmentPlane.setName ("FibulaPlane"+count);
      RigidBody segmentPlaneBody = new RigidBody ();
      segmentPlaneBody.setName ("CuttingPlane" + count);
      segmentPlaneBody.setSurfaceMesh (segmentPlane);
      RigidTransform3d transform = new maspack.matrix.RigidTransform3d ();
      transform.setRotation (Plane.getRotation ());
      transform.R.mulRotY180 ();
      transform.setTranslation (Plane.getPosition ());
      segmentPlane.transform (transform);
      // segmentPlaneBody.getRenderProps ().setVisible (false);
//      mechModel.add (segmentPlaneBody);
      PolygonalMesh resectionMesh =
         intersector
            .findDifference01 (
               Donor.getSurfaceMesh (), Plane.getSurfaceMesh ());
      // RigidBody resectionMeshBody = new RigidBody ("ResectionBody" + count);
      // resectionMeshBody.setSurfaceMesh (resectionMesh);
      // mechModel.add (resectionMeshBody);
      PolygonalMesh segment =
         intersector
            .findDifference01 (
               Donor.getSurfaceMesh (), segmentPlaneBody.getSurfaceMesh ());
      // Apparently this works for adding a mesh in relative space
      RigidTransform3d temptransform = segmentPlane.getMeshToWorld ();
      Donor.addMesh (segmentPlane);
//      Donor.addMesh(metalinsertlong);
      segmentPlane.inverseTransform (temptransform);
      Donor.setSurfaceMesh (resectionMesh);
      temptransform = resectionMesh.getMeshToWorld ();
      resectionMesh.inverseTransform (temptransform);
      FixedMeshBody ClippedDonor = new FixedMeshBody ();
      ClippedDonor.setName ("Segment" + count);
      ClippedDonor.setMesh (segment);
      mechModel.add (ClippedDonor);
      return null;

   }

   public PolygonalMesh createFibulaNonResect (
      MechModel mechModel, FixedMeshBody resectionFibulaMeshBody,
      FixedMeshBody nonResectionFibulaMeshBody,
      PolygonalMesh[] nonResectionFibulaMeshes,
      PolygonalMesh[] resectionFibulaMeshes, List<PolygonalMesh> cuttingPlanes,
      PolygonalMesh clippedDonorMesh,
      List<FixedMeshBody> cuttingPlanesMeshBodies,
      PolygonalMesh nonResectionFibulaMesh, RigidBody clippedDonorMeshBody) {

      nonResectionFibulaMeshes = new PolygonalMesh[cuttingPlanes.size () / 2];

      mechModel.removeMeshBody (resectionFibulaMeshBody);
      mechModel.removeMeshBody (nonResectionFibulaMeshBody);
      for (int i = 0; i < cuttingPlanes.size () / 2; i++) {
         nonResectionFibulaMeshes[i] = new PolygonalMesh ();
      }

      for (int i = 0; i < cuttingPlanes.size () - 1; i = i + 2) {
         PolygonalMesh firstPlane = cuttingPlanes.get (i);
         PolygonalMesh secondPlane = cuttingPlanes.get (i + 1);
         FixedMeshBody firstPlaneBody = cuttingPlanesMeshBodies.get (i);
         FixedMeshBody secondPlaneBody = cuttingPlanesMeshBodies.get (i + 1);

         Vector3d translateOrig1 = new Vector3d ();
         Vector3d translateOrig2 = new Vector3d ();
         firstPlane.computeCentroid (translateOrig1);
         secondPlane.computeCentroid (translateOrig2);

         correctiveTransformPlane (firstPlane, firstPlaneBody);
         correctiveTransformPlane (secondPlane, secondPlaneBody);

         SurfaceMeshIntersector intersector =
            new maspack.collision.SurfaceMeshIntersector ();

         // Create planes whose normals are opposite of cutting planes
         PolygonalMesh reversePlane1Mesh = MeshFactory.createPlane (90, 90);

         Point3d centroidPlane1 = new Point3d ();
         centroidPlane1.x = translateOrig1.x;
         centroidPlane1.y = translateOrig1.y;
         centroidPlane1.z = translateOrig1.z;
         meshHelper
            .createPlane (
               reversePlane1Mesh.getNormal (0),
               firstPlane.getNormal (0).negate (), centroidPlane1,
               firstPlaneBody.getPose (), reversePlane1Mesh);

         PolygonalMesh reversePlane2Mesh = MeshFactory.createPlane (90, 90);

         Point3d centroidPlane2 = new Point3d ();
         centroidPlane2.x = translateOrig2.x;
         centroidPlane2.y = translateOrig2.y;
         centroidPlane2.z = translateOrig2.z;
         meshHelper
            .createPlane (
               reversePlane2Mesh.getNormal (0),
               secondPlane.getNormal (0).negate (), centroidPlane2,
               secondPlaneBody.getPose (), reversePlane2Mesh);

         // Differencing reverse plane from first mesh
         intersector = new maspack.collision.SurfaceMeshIntersector ();

         nonResectionFibulaMeshes[i / 2] =
            intersector.findDifference01 (clippedDonorMesh, reversePlane1Mesh);

         // Differencing reverse plane from second mesh
         clippedDonorMesh =
            intersector.findDifference01 (clippedDonorMesh, reversePlane2Mesh);
      }

      // Combining all nonresected fibula mesh segments
      nonResectionFibulaMesh = new PolygonalMesh (clippedDonorMesh);
      nonResectionFibulaMesh.addMesh (clippedDonorMesh);
      for (int j = 0; j < nonResectionFibulaMeshes.length; j++) {
         nonResectionFibulaMesh =
            MeshFactory
               .getUnion (nonResectionFibulaMesh, nonResectionFibulaMeshes[j]);
      }
      nonResectionFibulaMeshBody =
         new FixedMeshBody ("Unresected Fibula", nonResectionFibulaMesh);
      // mechModel.addMeshBody (nonResectionFibulaMeshBody);

      clippedDonorMeshBody.getRenderProps ().setVisible (false);
      return nonResectionFibulaMesh;
   }

   public void correctiveTransform (
      PolygonalMesh plane, PolygonalMesh reversePlane) {
      Vector3d originalCentroidPlane = new Vector3d ();
      plane.computeCentroid (originalCentroidPlane);
      Vector3d newCentroid = new Vector3d ();
      reversePlane.computeCentroid (newCentroid);

      RigidTransform3d correctiveTranslation = new RigidTransform3d ();
      correctiveTranslation
         .setTranslation (
            new Vector3d (originalCentroidPlane).sub (newCentroid));
      reversePlane.transform (correctiveTranslation);
      reversePlane.setMeshToWorld (plane.getMeshToWorld ());
   }

   public void addPolygonalMeshToModel (
      MechModel mechModel, PolygonalMesh pm, String name) {
      FixedMeshBody fmb = new FixedMeshBody (name, pm);
      mechModel.addMeshBody (fmb);
   }

   public void correctiveTransformPlane (
      PolygonalMesh pm, FixedMeshBody... fmb) {
      Vector3d planeCentroid = new Vector3d ();
      pm.computeCentroid (planeCentroid);

      RigidTransform3d transform = new maspack.matrix.RigidTransform3d ();
      transform.setTranslation (planeCentroid);
      if (fmb.length > 0) {
         fmb[0].setPose (transform);
      }
      pm.inverseTransform (transform);

      pm.computeCentroid (planeCentroid);
   }
}
