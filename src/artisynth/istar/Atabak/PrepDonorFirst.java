/**
 *Author of PrepareDonorFirst: Atabak Eghbal (UBC)
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

public class PrepDonorFirst {

   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   RigidBody resectionMeshRigidBody = null;
   SurfaceMeshIntersector intersector =
      new maspack.collision.SurfaceMeshIntersector ();


   public List<Object> prepareDonorv2 (
      Point3d disPoint, Float DonorDistanceProx, Float DonorDistanceDis,
      PolygonalMesh DonorCuttingPlanes[], PolygonalMesh clippedDonorMesh,
      PolygonalMesh DonorMesh, FixedMeshBody DonorMeshBody){
      
      RigidTransform3d poseDonor = DonorMeshBody.getPose ();
      Point3d fibcen = new Point3d ();
      DonorMesh.computeCentroid (fibcen);
      fibcen.transform (poseDonor);
      System.out.println("Donor Center:");
      System.out.println(fibcen);   
      //------------------------------------------Bounding box ----------------------------------------
      OBB boundingbox = DonorMesh.computeOBB ();
      System.out.println("OBB boundingbox:");
      System.out.println(boundingbox);  
      
      RigidTransform3d posebb = new RigidTransform3d ();
      boundingbox.getTransform (posebb);
      Vector3d widths = new Vector3d ();
      boundingbox.getHalfWidths (widths);
      // widths.transform(posebb);
      System.out.println("BoundingBox Width:");
      System.out.println(widths);
      
      Point3d bbcenter = new Point3d ();
      boundingbox.getCenter (bbcenter);
      bbcenter.transform (poseDonor);
      
      System.out.println("BoundingBox Center:");
      System.out.println(bbcenter);
      
      Vector3d[] axes = new Vector3d[3];
      for (int i = 0; i < axes.length; i++) {
         axes[i] = new Vector3d ();
      }
      boundingbox.getSortedAxes (axes);    //returning the axes of the box 
      // all the transformations so far has been on the center of the bounding box, not on the box itself
      
      //let's have a look at the axes: 
      
      System.out.println("BoundingBox Axes:");
      for (int i = 0; i < 3; i++) {
         axes[i].transform (poseDonor);
          System.out.println(axes[i]);
      }
      
      Vector3d vec = new Vector3d (axes[0].copy ().negate ()); //hmmmmmmm? Because it's the largest axis, and it's not x actually it's z
      System.out.println("boundbox dir: ");
      System.out.println(vec);
      
      // Finding the largest width of the bounding box
      
      double width = 0.0;
      for (int i = 0; i < 3; i++) {
         if (widths.get (i) > width) {
            width = widths.get (i);
         }
      }
      // ------------------------------------------Bounding box ----------------------------------------
      
      // Finding the proximal and distal points ----------------
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
      //------------------------------------------------------------------------------------
      
      Vector3d dir = new Vector3d (vec);
      dir.normalize ();
      Vector3d proxTranslationVector = new Vector3d (dir.copy ());
      Vector3d disTranslationVector = new Vector3d (dir.copy ());
      
      
      // Working on plane closest to proximal end ------------------------Translating the proximal end given user inputs
      RigidTransform3d proxTransform = new RigidTransform3d ();
      // Scale translation Vector by value inputed by user
      proxTranslationVector.scale (DonorDistanceProx);
      proxTransform.setTranslation (proxTranslationVector);
      proxPoint.transform (proxTransform);
      //-------------------------------------------------------------------------------------------------------
      
      
      //Creating rectangular mesh for the Donor cutting planes
      DonorCuttingPlanes[0] = MeshFactory.createPlane (90, 90, 30, 30);   
      
      FixedMeshBody Donorclipping1 = new FixedMeshBody ("Donor Cutting Plane 0", DonorCuttingPlanes[0]);
      RigidTransform3d poseclipping0 = Donorclipping1.getPose ();
      Vector3d normal1 = DonorCuttingPlanes[0].getNormal (0);
      meshHelper
         .createPlane (
            normal1, dir, proxPoint, poseclipping0, DonorCuttingPlanes[0]);
      
      
      // Working on plane closest to distal end ---------------------- Translating the distal end given user inputs
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
      
      // So we created planes using the normal of the Donorcuttingplane, the Direction of the bounding box, the distal and prox points, the 4x4 transformation matrix of the DonorCuttingPlane, 
      //and the polygonal mesh of the DonorCuttingPlane
      
      clippedDonorMesh =
            intersector.findDifference01 (DonorMesh, DonorCuttingPlanes[0]);
      
      clippedDonorMesh =
            intersector
               .findDifference01 (clippedDonorMesh, DonorCuttingPlanes[1]);
      
      
      List<Object> returnList = new ArrayList<Object> ();
      returnList.add (disPoint);
      returnList.add (DonorCuttingPlanes);
      returnList.add (clippedDonorMesh);

      return returnList;
      }
   
  
}
