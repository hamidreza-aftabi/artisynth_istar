package artisynth.istar.MandibleRecon;

import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractAction;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.RigidMeshComp;
import artisynth.istar.Prisman.HelperMandibleFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ImprovedFormattedTextField;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import artisynth.istar.Assist.Assist;



public class ManualFibGuide extends AbstractAction {
   ReconstructionModel root;
   MechModel mechModel;
   Assist Assist = new Assist();
   HelperMandibleFunctions mandHelper = new HelperMandibleFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   maspack.matrix.Point3d disPoint = new maspack.matrix.Point3d ();
   PolygonalMesh DonorCuttingPlanes[] = new PolygonalMesh[2];
   
   public ManualFibGuide(ReconstructionModel root, MechModel model) {
      this.root = root;
      this.mechModel = model;
   }
   
   @Override
   public void actionPerformed (ActionEvent e) {

      Point3d mandcent = new Point3d ();
      Assist.GetMesh (mechModel, "Mandible").computeCentroid (mandcent);
      RigidBody clippedDonorMeshBody = Assist.GetRigidBody (mechModel, "ClippedDonor");
      PolygonalMesh clippedDonorMesh = clippedDonorMeshBody.getSurfaceMesh ();
      
      RigidTransform3d posefib =
         Assist.GetRigidBody (mechModel, "ClippedDonor").getPose ();
      
      System.out.println ("Begin Creating Donor Guide");

      // Generating Information from ClippedDonor
      OBB Donorbb = Assist.GetMesh (mechModel, "ClippedDonor").computeOBB ();
      Vector3d[] axis = new Vector3d[3];
      for (int i = 0; i < axis.length; i++) {
         axis[i] = new Vector3d ();
      }
      
      Vector3d[] axes = new Vector3d[3];
      for (int i = 0; i < axes.length; i++) {
         axes[i] = new Vector3d ();
      }
      Donorbb.getSortedAxes (axes);
      Vector3d lengthdir = new Vector3d (axes[0].copy ());
      lengthdir.transform (posefib);
      root.DonorLengthDir = new VectorNd (lengthdir);
      
      System.out.println (root.DonorLengthDir);
      Vector3d fibDir = new Vector3d (root.DonorLengthDir);

      FrameMarker[] DonorLineFrameMarkers =
      clippedDonorMeshBody.getFrameMarkers ();
      OBB boundingbox = clippedDonorMeshBody.getSurfaceMesh ().computeOBB ();
      
      Vector3d widths = new Vector3d ();
      boundingbox.getHalfWidths (widths);
      Vector3d bcenter = new Vector3d ();
      boundingbox.getCenter (bcenter);
      Point3d bbcenter = new Point3d (bcenter);
      bbcenter.transform (posefib);

      double fibulaLength = 0.0;
      for (int i = 0; i < 3; i++) {
         if (widths.get (i) > fibulaLength) {
            fibulaLength = widths.get (i);
         }
      }
      
   // topPoint and botPoint are top and bottom points on clipped donor
      double x = root.DonorLengthDir.get (0);
      double y = root.DonorLengthDir.get (1);
      double z = root.DonorLengthDir.get (2);
      x = x * fibulaLength + bbcenter.get (0);
      y = y * fibulaLength + bbcenter.get (1);
      z = z * fibulaLength + bbcenter.get (2);
      Point3d botPoint = new Point3d (x, y, z);
      x = root.DonorLengthDir.get (0) * (-1);
      y = root.DonorLengthDir.get (1) * (-1);
      z = root.DonorLengthDir.get (2) * (-1);
      x = x * fibulaLength + bbcenter.get (0);
      y = y * fibulaLength + bbcenter.get (1);
      z = z * fibulaLength + bbcenter.get (2);
      root.topPoint.set (x, y, z);

      // Calculating and applying transform
      Vector3d normProx =
         new Vector3d (DonorCuttingPlanes[0].getNormal (0).copy ());
      normProx.normalize ();

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // Setting up the pair of cutting planes that will be transformed for
      // each Donor segment
      PolygonalMesh[] DonorPiecesClippingPlanes = new PolygonalMesh[2];
      if (disPoint.distance (root.topPoint) > disPoint.distance (botPoint)) {
         DonorPiecesClippingPlanes[0] =
            MeshFactory.createPlane (90, 90, 10, 10);
         FixedMeshBody Donorclipping1 =
            new FixedMeshBody (
               "Donor Pieces 0", DonorPiecesClippingPlanes[0]);
         RigidTransform3d poseclipping0 = Donorclipping1.getPose ();
         Vector3d normal1 = DonorPiecesClippingPlanes[0].getNormal (0);
         meshHelper
            .createPlane (
               normal1, new Vector3d (normProx), new Point3d (0, 0, 0),
               poseclipping0, DonorPiecesClippingPlanes[0]);

         DonorPiecesClippingPlanes[1] =
            MeshFactory.createPlane (90, 90, 10, 10);
         FixedMeshBody Donorclipping2 =
            new FixedMeshBody (
               "Donor Pieces 1", DonorPiecesClippingPlanes[1]);
         poseclipping0 = Donorclipping2.getPose ();
         normal1 = DonorPiecesClippingPlanes[1].getNormal (0);
         meshHelper
            .createPlane (
               normal1, new Vector3d (normProx.copy ().negate ()),
               new Point3d (0, 0, 0), poseclipping0,
               DonorPiecesClippingPlanes[1]);
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

      }

      // Projecting first placed DonorLine Marker onto DonorLengthDir
      VectorNd frameMarkerProjOntoCenterline = new VectorNd ();
      VectorNd AP =
         new VectorNd (DonorLineFrameMarkers[0].getPosition ())
            .sub (new VectorNd (root.topPoint.copy ()));
      VectorNd AB =
         new VectorNd (new VectorNd (botPoint.copy ()))
            .sub (new VectorNd (root.topPoint.copy ()));
      frameMarkerProjOntoCenterline =
         new VectorNd (new VectorNd (root.topPoint.copy ()))
            .add (
               new VectorNd (AB)
                  .scale (
                     new VectorNd (AP).dot (AB)
                     / new VectorNd (AB).dot (AB)));

      root.fromCenterToSurface =
         new VectorNd (DonorLineFrameMarkers[0].getPosition ());
      root.fromCenterToSurface.sub (frameMarkerProjOntoCenterline);
      VectorNd storageVariable = new VectorNd (root.fromCenterToSurface);
      
      root.fromCenterToSurface.normalize ();
      Vector3d cross =
         new Vector3d (fibDir)
            .cross (
               new Vector3d (
                  root.fromCenterToSurface.get (0),
                  root.fromCenterToSurface.get (1),
                  root.fromCenterToSurface.get (2)));
      
      Vector3d slotDir = new Vector3d (cross);
      slotDir.normalize ();
      
      widths = new Vector3d ();
      Donorbb.getWidths (widths);
      
      Point3d centclipped = new Point3d ();
      Assist.GetMesh (mechModel, "ClippedDonor").computeCentroid (centclipped);
      centclipped.transform (posefib);

      Vector3d normalFib = new Vector3d (slotDir);
      normalFib.normalize ();

      double max = 0.0;
      for (int i = 0; i < 3; i++) {
         if (widths.get (i) > max) {
            max = widths.get (i);
         }
      }

      // Creating Cylinder that will cut out the square base. Diameter is
      // based on the width of the clipped donor
      Donorbb = Assist.GetMesh (mechModel, "ClippedDonor").computeOBB ();
      widths = new Vector3d ();
      Donorbb.getWidths (widths);
      Vector3d halfw = new Vector3d ();
      Donorbb.getHalfWidths (halfw);
      // System.out.println(widths);
      // System.out.println(halfw);

      double height = 0.0;
      int hidx = 0;
      for (int i = 0; i < 3; i++) {
         if (widths.get (i) > height) {
            hidx = i;
            height = widths.get (i);
         }
      }

      double min = 10000000.0;
      int midx = 0;
      for (int i = 0; i < 3; i++) {
         if (widths.get (i) < min) {
            midx = i;
            min = widths.get (i);
         }
      }

      double diameter = 0.0;
      for (int i = 0; i < 3; i++) {
         if ((i != hidx) && (i != midx)) {
            diameter = widths.get (i);
         }
      }

      double radius = diameter / 2 + 2;
      
      PolygonalMesh cylinder =
      MeshFactory.createCylinder (radius, (height + 20), 40, 1, 35);
      FixedMeshBody cbody = new FixedMeshBody ("Cylinder", cylinder);

      // Creating and Transforming Base so that it is aligned with the
      // clippedDonor
      double length = max / 2;
      PolygonalMesh Base =
         MeshFactory.createBox (max, 24, 12, mandcent, 35, 8, 4);
      FixedMeshBody BaseBody = new FixedMeshBody ("Base", Base);

      RigidTransform3d basePose = BaseBody.getPose ();

      OBB basebb = Base.computeOBB ();
      basebb.getSortedAxes (axis);
      Vector3d longest = new Vector3d (axis[0].copy ());
      Vector3d normalbase = new Vector3d (axis[1].copy ());
      Point3d basecenter = new Point3d ();
      basebb.getCenter (basecenter);
      longest.transform (basePose);
      longest.normalize ();
      normalbase.transform (basePose);
      normalbase.normalize ();
      basecenter.transform (basePose);

      Point3d p1 = new Point3d (basecenter);
      Point3d p2 = new Point3d (basecenter);
      Vector3d ext = new Vector3d (longest);
      ext.scale (length);
      p1.add (ext);
      p2.sub (ext);
      Point3d[] PFTBase =
         meshHelper
            .pointsForTransform (p1, p2, new Vector3d (normalbase.copy ()));

      p1 = new Point3d (centclipped);
      p2 = new Point3d (centclipped);
      Vector3d toside = new Vector3d (root.fromCenterToSurface);
      toside.normalize ();
      toside.scale (radius - 4);
      p1.add (toside);
      p2.add (toside);
      ext = new Vector3d (fibDir);
      ext.normalize ();
      ext.scale (length);
      p1.add (ext);
      p2.sub (ext);
      Point3d[] PFTFib =
         meshHelper
            .pointsForTransform (p1, p2, new Vector3d (normalFib.copy ()));
   
      RigidTransform3d tBase = meshHelper.SVDRegistration (PFTFib, PFTBase);

      Base.transform (tBase);

      // Cut base to the appropriate length
      basePose = BaseBody.getPose ();
      Point3d basecentroid = new Point3d ();
      Base.computeCentroid (basecentroid);
      basecentroid.transform (basePose);

      double actualLength = root.topmax.distance (root.botmax);
      if (max > (actualLength + 50)) {
         Vector3d planenormal = new Vector3d (fibDir);
         PolygonalMesh baseClipPlane = MeshFactory.createPlane (90, 90, 15, 15);
         FixedMeshBody baseClipPlaneBody =
            new FixedMeshBody ("Base Clip Plane", baseClipPlane);
         RigidTransform3d posePlane = baseClipPlaneBody.getPose ();
         Vector3d normalactual = baseClipPlane.getNormal (0);
         double trimlength = (actualLength + 50) - (max / 2);
         Point3d trimVector = new Point3d (planenormal.copy ());
         trimVector.scale (trimlength);
         Point3d planecenter = new Point3d (basecentroid);
         planecenter.add (trimVector);
         meshHelper
            .createPlane (
               normalactual, new Vector3d (planenormal).negate (), planecenter,
               posePlane, baseClipPlane);

         SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();
         Base = intersector.findDifference01 (Base.clone (), baseClipPlane);

         baseClipPlaneBody.getRenderProps ().setVisible (false);
      }

      // Transforming Cylinder so that it is aligned with clippedDonor
      RigidTransform3d poseC = cbody.getPose ();

      OBB cbb = cylinder.computeOBB ();
      Vector3d[] caxis = new Vector3d[3];
      for (int i = 0; i < 3; i++) {
         caxis[i] = new Vector3d ();
      }
      cbb.getSortedAxes (caxis);

      Vector3d cylinderDir = new Vector3d (caxis[0].copy ());
      cylinderDir.transform (poseC);

      RotationMatrix3d rotateCylinder =
         meshHelper.rotatePlane (cylinderDir, fibDir);
      AffineTransform3d rotateC = new AffineTransform3d ();
      rotateC.setRotation (rotateCylinder);
      cylinder.transform (rotateC);

      Base.computeCentroid (basecenter);
      RigidTransform3d baseRT = BaseBody.getPose ();
      basecenter.transform (baseRT);
      Point3d newLocation = new Point3d (centclipped);
      Vector3d extL = new Vector3d (root.fromCenterToSurface);
      extL.normalize ();
      extL.scale (4);
      newLocation.sub (extL);
      meshHelper.setPlaneOrigin (cylinder, newLocation);

      // Create Donor Guide using the Rectangular Base
      PolygonalMesh newBase = new PolygonalMesh (Base);
      RigidBody xyz = new RigidBody ("Base");
      xyz.setSurfaceMesh (newBase);
      mechModel.add (xyz);

      PolygonalMesh DonorGuide = new PolygonalMesh (newBase.clone ());
      
      
      
      
      
      
      
//      // TODO Auto-generated method stub
//      RigidBody manualRecon = Assist.GetRigidBody (model, "DonorRigidBody");
//      ArrayList<RigidMeshComp> planes = new ArrayList<>();
//      for (RigidMeshComp mesh: manualRecon.getMeshComps ()) {
//         if(mesh.getName ().contains ("FibulaPlane")) {
//            planes.add (mesh);  
//         }
//      }
//      
//      System.out.println(planes.size ());
//      int count = 0;
//      for (RigidMeshComp plane: planes) {
////         PolygonalMesh box = MeshFactory.createBox (40, 20, 20);
////         RigidBody boxbody = new RigidBody();
////         boxbody.setSurfaceMesh (box);
////         boxbody.transformGeometry (plane.getMesh ().XMeshToWorld);
////         boxbody.transformGeometry (plane.getRigidBody ().getPosition ());
////         boxbody.transformPose(plane.getRigidBody ().getPose ());
////         model.addRigidBody (boxbody);
//      }
   }
   
}