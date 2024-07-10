package artisynth.istar.Prisman.undo;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import artisynth.core.gui.editorManager.Command;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMandibleFunctions;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ImprovedFormattedTextField;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.BVIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class TestingTransform implements Command {
   Vector3d scapulaNormalAvg = new Vector3d ();
   FixedMeshBody nonResectionMeshBody;
   List<PolygonalMesh> donorSegmentMeshes = new ArrayList<PolygonalMesh> ();
   List<FixedMeshBody> donorSegmentBodies = new ArrayList<FixedMeshBody> ();
   List<PolygonalMesh> mandiblePlanes = new ArrayList<PolygonalMesh> ();
   List<Point3d> segmentCentroids = new ArrayList<Point3d> ();
   PolygonalMesh DonorCuttingPlanes[] = new PolygonalMesh[2];
   List<Point3d> lineTops = new ArrayList<Point3d> ();
   List<Point3d> lineBottoms = new ArrayList<Point3d> ();
   List<Vector3d> lengthDirs = new ArrayList<Vector3d> ();
   List<Vector3d> toSides = new ArrayList<Vector3d> ();
   List<PolygonalMesh> donorSegmentMeshesShort =
      new ArrayList<PolygonalMesh> ();
   List<FixedMeshBody> donorSegmentBodiesShort =
      new ArrayList<FixedMeshBody> ();
   List<Point3d> spoints = new ArrayList<Point3d> ();
   Point3d disPoint = new Point3d ();
   FixedMeshBody resectionFibulaMeshBody;
   FixedMeshBody nonResectionFibulaMeshBody;
   ReconstructionModel root = null;
   Assist Assist = new Assist ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMandibleFunctions mandHelper = new HelperMandibleFunctions ();
   MechModel mechModel = null;
   PolygonalMesh[] nonResectionFibulaMeshes;
   PolygonalMesh[] resectionFibulaMeshes;
   ImprovedFormattedTextField DonorDistanceProx;
   ImprovedFormattedTextField DonorDistanceDis;
   ImprovedFormattedTextField multiplier;
   double SAW_BLD_THICK;

   public TestingTransform (ReconstructionModel root, MechModel mechModel,
   ImprovedFormattedTextField DonorDistanceProx,
   ImprovedFormattedTextField DonorDistanceDis,
   ImprovedFormattedTextField multiplier, double SAW_BLD_THICK) {
      this.root = root;
      this.mechModel = mechModel;
      this.DonorDistanceDis = DonorDistanceDis;
      this.DonorDistanceProx = DonorDistanceProx;
      this.multiplier = multiplier;
      this.SAW_BLD_THICK = SAW_BLD_THICK;
   }

   //automated scapula solutions- for optimizing fibula solution
   
   @Override
   public void execute() {
      PolygonalMesh nonResectionFibulaMesh = new PolygonalMesh ();
      RigidBody clippedDonorMeshBody = new RigidBody ("ClippedDonor");
      PolygonalMesh clippedDonorMesh = new PolygonalMesh ();
      List<Object> returnObjects =
         this.mandHelper
            .prepareDonor (
               disPoint, Float.parseFloat (DonorDistanceProx.getText ()),
               Float.parseFloat (DonorDistanceDis.getText ()),
               this.DonorCuttingPlanes, clippedDonorMesh,
               Assist.GetMesh (mechModel, "Donor"), root.scapulaTrimPlane,
               root.donorIsScapulaCheckBox.isSelected (),
               Assist.GetMeshBody (mechModel, "Donor"), false);
      clippedDonorMeshBody = new RigidBody ("ClippedDonor");
      disPoint = (Point3d)returnObjects.get (0);
      DonorCuttingPlanes = (PolygonalMesh[])returnObjects.get (1);
      clippedDonorMesh = (PolygonalMesh)returnObjects.get (2);
      clippedDonorMeshBody.setSurfaceMesh (clippedDonorMesh);
      
      if (Assist.GetRigidBody (mechModel, "ClippedDonor") == null) {
         mechModel.addRigidBody (clippedDonorMeshBody);
         Assist
            .GetMeshBody (mechModel, "Donor").getRenderProps ()
            .setVisible (false);
      }
      else {
         clippedDonorMeshBody = Assist.GetRigidBody (mechModel, "ClippedDonor");
         clippedDonorMesh = clippedDonorMeshBody.getSurfaceMesh ();
         FrameMarker[] frameMarkers = clippedDonorMeshBody.getFrameMarkers ();
         
         //positioning top plane
         //normal of mand plane
         //vector formed by a point on the plane and a point to the nearest RDP point
         //put a new plane w/ vector (0,0,-1) on fibula
         //transform it to tip at an angle equal to the angle between normal of mand plane and rdp vec
         
         Point3d fm1 = frameMarkers[0].getLocation ();
         
         List<PolygonalMesh> mandiblePlanes = new ArrayList<PolygonalMesh> ();
         mandiblePlanes.add (Assist.GetMesh (mechModel, "Plane1"));
         mandiblePlanes.add (Assist.GetMesh (mechModel, "Plane2"));
                  
         PolygonalMesh plane1Mand = mandiblePlanes.get (0);
         
         
         Vector3d normCompare = plane1Mand.getNormal (0);
         
         Point3d[] simpPoints = new Point3d[root.simpList.getNumKnots ()];
         Iterator<NumericListKnot> simpItr = root.simpList.iterator ();
         int l = 0;
         while (simpItr.hasNext ()) {
            simpPoints[l] = new Point3d (simpItr.next ().v);
            l++;
         }
         
         Point3d p1 = simpPoints[0];
         Point3d p2 = simpPoints[1];
         Vector3d vecCompare = p2.sub (p1);
         System.out.println ("dot product: " + vecCompare.dot (normCompare));
         System.out.println ("VEC1 norm value: " + vecCompare.norm());
         System.out.println ("Norm 2: " + normCompare.norm());
         
         
         
//         //set origin of plane1Mand to p1
//         meshHelper.setPlaneOrigin(plane1Mand, p1);
//         //
         Vector3d help = p1.add (normCompare);
         
         double cosCompare = (vecCompare.dot (help)) / (vecCompare.norm () * help.norm ());
         Vector3d clippingPlane1Norm = DonorCuttingPlanes[0].getNormal (0);
         
         System.out.println ("THETA between RDP and Input Plane1: " + (Math.acos (cosCompare) * 180/Math.PI));

         //translate plane to first frameMarker
         //n = clippingPlane1Norm
         //get end of n
         //use another framemarker to get vector u, with end of "n" as start of vector u
         //n x u = x 
         //cos-1 theta to get theta
         //scale x to be tan(theta)*n.norm()
         
         PolygonalMesh topPlane = MeshFactory.createPlane (90, 90, 10, 10);
         FixedMeshBody topPlaneMesh = new FixedMeshBody (topPlane);
         meshHelper.createPlane 
            (topPlane.getNormal (0), clippingPlane1Norm, fm1, topPlaneMesh.getPose (), topPlane);

         RigidBody topTemp = new RigidBody ("Testing Plane");
         topTemp.setSurfaceMesh (topPlane);
         mechModel.addRigidBody (topTemp);
         
         //getting end of n
         Vector3d n = clippingPlane1Norm;
         double lengthN = n.norm ();
         Point3d Q = (Point3d) fm1.add (n);
         
         //getting n'
         Vector3d nP = n.negate ();
         
         //getting u
         Point3d fm2 = frameMarkers[1].getLocation ();
         Vector3d u = fm2.sub (Q);
         
         //getting x (unkonwn length) and making it unit length
         Vector3d x = u.cross (nP);
         x = x.normalize ();
         
         //find scale factor s
         double theta = Math.acos (cosCompare);
         double s = Math.tan (Math.PI/2 - theta) * n.norm ();
         
         //scale x
         x = x.scale (s);
         
         Vector3d topPlaneNewNorm = n.add (x);
         
         PolygonalMesh finalPlane = MeshFactory.createPlane (90, 90, 10, 10);
         FixedMeshBody finalPlaneBody = new FixedMeshBody (finalPlane);
//         meshHelper.createPlane 
//            (finalPlane.getNormal (0), topPlaneNewNorm, fm1, finalPlaneBody.getPose (), finalPlane);
         
//       RigidTransform3d poseTemp = new RigidTransform3d ();
//       poseTemp.setXyz (fm1.x, fm1.y, fm1.z);
//       meshHelper.createPlane 
//       (topPlane.getNormal (0), clippingPlane1Norm, fm1, poseTemp, topPlane);
         
//         
         double testingCos = topPlaneNewNorm.dot (n) / (topPlaneNewNorm.norm () * n.norm ());
         System.out.println ("Final Plane angle: " + (Math.acos (testingCos) * 180/Math.PI));
         
         RotationMatrix3d rotateFinalPlane = meshHelper.rotatePlane (finalPlane.getNormal (0), topPlaneNewNorm);
         AffineTransform3d affineRotateFinal = new AffineTransform3d ();
         affineRotateFinal.setRotation (rotateFinalPlane);
         finalPlane.transform (affineRotateFinal);
         meshHelper.setPlaneOrigin (finalPlane, fm1);
         
         
         RigidBody trying = new RigidBody ("Testing Plane Rotated");
         trying.setSurfaceMesh (finalPlane);
         mechModel.addRigidBody (trying);
         

         
         
         
         
         
         
         
         
         
         
      }
      

   }

   @Override
   public void undo () {
   
   }

   @Override
   public String getName () {
      // TODO Auto-generated method stub
      return "Testing Transform";
   }
}
