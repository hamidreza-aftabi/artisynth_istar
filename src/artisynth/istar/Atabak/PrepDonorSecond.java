/**
 *Author of PrepareDonorSecond: Atabak Eghbal (UBC)
 */
package artisynth.istar.Atabak;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
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
import maspack.matrix.VectorNd;

public class PrepDonorSecond {
   
   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   RigidBody resectionMeshRigidBody = null;
   SurfaceMeshIntersector intersector =
      new maspack.collision.SurfaceMeshIntersector ();


   public List<Object> generateSegmentVars (
      Point3d disPoint, PolygonalMesh DonorCuttingPlanes[],
      PolygonalMesh clippedDonorMesh,RigidBody clippedDonorMeshBody, VectorNd DonorLengthDir,
      Point3d topPoint, VectorNd fromCenterToSurface){
      
          
      System.out.println ("Begin Transforming");

      
      OBB boundingbox = clippedDonorMeshBody.getSurfaceMesh ().computeOBB ();
      RigidTransform3d poseDonor = clippedDonorMeshBody.getPose ();
      Vector3d[] axes = new Vector3d[3];
      for (int i = 0; i < axes.length; i++) {
         axes[i] = new Vector3d ();
      }
      boundingbox.getSortedAxes (axes);
      Vector3d lengthdir = new Vector3d (axes[0].copy ());
      lengthdir.transform (poseDonor);
      DonorLengthDir = new VectorNd (lengthdir);
      Point3d DonorCentroid = new Point3d ();
      clippedDonorMeshBody.getSurfaceMesh ().computeCentroid (DonorCentroid);
      DonorCentroid.transform (poseDonor);

      Vector3d widthBB = new Vector3d ();
      boundingbox.getWidths (widthBB);
      System.out.println("widthBB:");
      System.out.println(widthBB);
      PolygonalMesh boundingClip =
         MeshFactory
            .createBox (widthBB.get (0), widthBB.get (1), widthBB.get (2));
      boundingClip.transform (boundingbox.getTransform ());
      FixedMeshBody bCb = new FixedMeshBody ("boundingClip", boundingClip);

      DonorLengthDir.normalize ();

      // Checking direction of DonorLengthDir to make sure that it is
      // pointing in the direction of:
      // proximal to distal for fibula, distal to proximal for scapula
      
      FrameMarker[] DonorLineFrameMarkers = clippedDonorMeshBody.getFrameMarkers ();
      
      
      Vector3d upDir =
         new Vector3d (DonorLineFrameMarkers[0].getPosition ());
      
      System.out.println("DonorLengthDir:");
      System.out.println(DonorLengthDir);
      
      upDir.sub (DonorCentroid);
      double dotproduct = upDir.dot (DonorLengthDir);
      dotproduct =
         dotproduct / (upDir.norm () * DonorLengthDir.norm ());
      if (dotproduct > 0) {
         DonorLengthDir.negate ();
      }

      // Getting prox and distal endpoints
      Vector3d widths = new Vector3d ();
      boundingbox.getHalfWidths (widths);
      System.out.println("widths:");
      System.out.println(widths);
      Point3d bcenter = new Point3d ();
      boundingbox.getCenter (bcenter);
      Point3d bbcenter = new Point3d (bcenter);
      bbcenter.transform (poseDonor);

      double fibulaLength = 0.0;
      for (int i = 0; i < 3; i++) {
         if (widths.get (i) > fibulaLength) {
            fibulaLength = widths.get (i);
         }
      }

      // topPoint and botPoint are top and bottom points on clipped donor
      double x = DonorLengthDir.get (0);
      double y = DonorLengthDir.get (1);
      double z = DonorLengthDir.get (2);
      x = x * fibulaLength + bbcenter.get (0);
      y = y * fibulaLength + bbcenter.get (1);
      z = z * fibulaLength + bbcenter.get (2);
      Point3d botPoint = new Point3d (x, y, z);
      x = DonorLengthDir.get (0) * (-1);
      y = DonorLengthDir.get (1) * (-1);
      z = DonorLengthDir.get (2) * (-1);
      x = x * fibulaLength + bbcenter.get (0);
      y = y * fibulaLength + bbcenter.get (1);
      z = z * fibulaLength + bbcenter.get (2);
      topPoint.set (x, y, z);

      // Calculating and applying transform
      Vector3d normProx =
         new Vector3d (DonorCuttingPlanes[0].getNormal (0).copy ());
      normProx.normalize ();

    //--------------------------------------------------------------------------------------------------------------------  
   // Setting up the pair of cutting planes that will be transformed for
      // each Donor segment
      List<Object> returnList = new ArrayList<Object> ();
      returnList.add (topPoint);
      returnList.add (botPoint);
      
      PolygonalMesh[] DonorPiecesClippingPlanes = new PolygonalMesh[2];
      
      if (disPoint.distance (topPoint) > disPoint.distance (botPoint)) {
         
         DonorPiecesClippingPlanes[0] = MeshFactory.createPlane (90, 90, 10, 10);
         FixedMeshBody Donorclipping1 = new FixedMeshBody ("Donor Pieces 0", DonorPiecesClippingPlanes[0]);
         RigidTransform3d poseclipping0 = Donorclipping1.getPose ();
         Vector3d normal1 = DonorPiecesClippingPlanes[0].getNormal (0);
         meshHelper
            .createPlane (
               normal1, new Vector3d (normProx), new Point3d (0, 0, 0),
               poseclipping0, DonorPiecesClippingPlanes[0]);

         DonorPiecesClippingPlanes[1] = MeshFactory.createPlane (90, 90, 10, 10);
         FixedMeshBody Donorclipping2 = new FixedMeshBody ("Donor Pieces 1", DonorPiecesClippingPlanes[1]);
         poseclipping0 = Donorclipping2.getPose ();
         normal1 = DonorPiecesClippingPlanes[1].getNormal (0);
         meshHelper
            .createPlane (
               normal1, new Vector3d (normProx.copy ().negate ()),
               new Point3d (0, 0, 0), poseclipping0,
               DonorPiecesClippingPlanes[1]);
         
         
         returnList.add (DonorPiecesClippingPlanes);
      }
      else {
         DonorPiecesClippingPlanes[0] =
            MeshFactory.createPlane (90, 90, 10, 10);
         FixedMeshBody Donorclipping1 =
            new FixedMeshBody (
               "Donor Pieces 0", DonorPiecesClippingPlanes[0]);
         RigidTransform3d poseclipping0 = Donorclipping1.getPose ();
         Vector3d normal1 =
            new Vector3d (DonorPiecesClippingPlanes[0].getNormal (0));
         meshHelper
            .createPlane (
               normal1, new Vector3d (normProx.copy ().negate ()),
               new Point3d (0, 0, 0), poseclipping0,
               DonorPiecesClippingPlanes[0]);

         DonorPiecesClippingPlanes[1] =
            MeshFactory.createPlane (90, 90, 10, 10);
         FixedMeshBody Donorclipping2 =
            new FixedMeshBody (
               "Donor Pieces 1", DonorPiecesClippingPlanes[1]);
         poseclipping0 = Donorclipping2.getPose ();
         normal1 = new Vector3d (DonorPiecesClippingPlanes[1].getNormal (0));
         meshHelper
            .createPlane (
               normal1, new Vector3d (normProx), new Point3d (0, 0, 0),
               poseclipping0, DonorPiecesClippingPlanes[1]);
         
         
         returnList.add (DonorPiecesClippingPlanes);

      }
      //-------------------------------------------------------------------------------------------------------------------- 
      //-------------------------------------------------------------------------------------------------------------------- 
      // So, So far I've created 2 planes which are on the origin of the donor but have opposite normal directions
      
      
      // Projecting first placed DonorLine Marker onto DonorLengthDir
      VectorNd frameMarkerProjOntoCenterline = new VectorNd ();
      VectorNd AP = new VectorNd (DonorLineFrameMarkers[0].getPosition ()).sub (new VectorNd (topPoint.copy ()));
      VectorNd AB = new VectorNd (new VectorNd (botPoint.copy ())).sub (new VectorNd (topPoint.copy ()));
      //using this formula we're projecting the frame marker onto the DonorLengthDir line
      frameMarkerProjOntoCenterline = new VectorNd (new VectorNd (topPoint.copy ()))
            .add (
               new VectorNd (AB)
                  .scale (
                     new VectorNd (AP).dot (AB)
                     / new VectorNd (AB).dot (AB)));
      
      //Calculating the distance from the centerline of donor to the framemarker on the surface of the donor      
      fromCenterToSurface = new VectorNd (DonorLineFrameMarkers[0].getPosition ());
      fromCenterToSurface.sub (frameMarkerProjOntoCenterline);
      VectorNd storageVariable = new VectorNd (fromCenterToSurface);
      System.out.println ("fromCenterToSurface " + fromCenterToSurface);
      //----------------------------------------------------------------------------------------------------------------
      //----------------------------------------------------------------------------------------------------------------
      // Lowering srcPoint which will be the center of the top plane so that
      // when the first cutting plane is really slanted, it doesn't run pass
      // the proximal endpoint
      VectorNd srcPoint = new VectorNd ();
      srcPoint = new VectorNd (topPoint.copy ());
      srcPoint.add (fromCenterToSurface);
      double scaleLower = 10;


      VectorNd lower = new VectorNd (DonorLengthDir.copy ()).scale (scaleLower);
      System.out.println ("DonorLengthDir " + DonorLengthDir);
      System.out.println ("lower " + lower);
      srcPoint.add (lower);
      System.out.println ("srcPOint " + srcPoint);

      returnList.add (srcPoint);
      returnList.add (DonorLengthDir);
      returnList.add (lower);
      returnList.add (fromCenterToSurface);
      returnList.add (widths);
      return returnList;
      
   }
   
}