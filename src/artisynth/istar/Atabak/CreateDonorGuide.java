 package artisynth.istar.Atabak;


import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.Command;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.util.ArtisynthPath;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.BVIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class CreateDonorGuide implements Command {
   ReconstructionModel root = null;
   MechModel mechModel = null;
   String Target;
   Assist Assist = new Assist ();
   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
  
   public CreateDonorGuide(ReconstructionModel root,
      MechModel mechModel, String Target) {
      this.root = root;
      this.mechModel = mechModel;
      this.Target = Target;
   }
   @Override
   public void execute () {
      // TODO Auto-generated method stub

      // Initializing variable list that will be used later
      Point3d mandcent = new Point3d ();
      Assist.GetMesh (mechModel, Target).computeCentroid (mandcent);
      ArrayList<PolygonalMesh> large = new ArrayList<PolygonalMesh> ();
      ArrayList<FixedMeshBody> largeBodies = new ArrayList<FixedMeshBody> ();
      ArrayList<PolygonalMesh> small = new ArrayList<PolygonalMesh> ();
      ArrayList<FixedMeshBody> smallBodies = new ArrayList<FixedMeshBody> ();
      // List<Vector3d> slotDirScapula = new ArrayList<Vector3d> ();

      RigidTransform3d posefib =
         Assist.GetRigidBody (mechModel, "ClippedDonor").getPose ();

      System.out.println ("Begin Creating Donor Guide");

      // Generating Information from ClippedDonor
      OBB Donorbb = Assist.GetMesh (mechModel, "ClippedDonor").computeOBB ();
      Vector3d[] axis = new Vector3d[3];
      for (int i = 0; i < axis.length; i++) {
         axis[i] = new Vector3d ();
      }

      Vector3d fibDir = new Vector3d (root.DonorLengthDir);

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
      if (root.donorIsScapulaCheckBox.isSelected ()) {
         Vector3d initial = new Vector3d (slotDir);
         slotDir = new Vector3d (root.scapulaTrimPlane.getNormal (0));
         slotDir
            .transform (
               Assist.GetMeshBody (mechModel, "ScapulaTrimPlane").getPose ());
         slotDir.normalize ();
         if (slotDir.dot (initial) < 0) {
            slotDir.negate ();
         }
      }

      Vector3d widths = new Vector3d ();
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

      PolygonalMesh DonorGuide = new PolygonalMesh (newBase.clone ());
      large.clear ();
      small.clear ();
      largeBodies.clear ();
      smallBodies.clear ();

      // Create marking. Mark indicate the side that is the
      // proximal/first plated side of the mandible
      Point3d centreGuide = new Point3d ();
      DonorGuide.computeCentroid (centreGuide);

      PolygonalMesh L =
         MeshFactory
            .createBox (
               3, 5, 15, centreGuide.get (0), centreGuide.get (1),
               centreGuide.get (2));
      FixedMeshBody LB = new FixedMeshBody ("Marking", L);
      OBB LBB = L.computeOBB ();
      Vector3d[] Laxis = new Vector3d[3];
      for (int i = 0; i < 3; i++) {
         Laxis[i] = new Vector3d ();
      }
      LBB.getSortedAxes (Laxis);
      RigidTransform3d LPose = LB.getPose ();
      Point3d Lcentroid = new Point3d ();
      L.computeCentroid (Lcentroid);
      Lcentroid.transform (LPose);
      for (int i = 0; i < 3; i++) {
         Laxis[i].transform (LPose);
      }
      Vector3d Ldir = new Vector3d (Laxis[0].copy ());
      Vector3d Lnorm = new Vector3d (Laxis[2].copy ());
      Point3d l1 = new Point3d (Lcentroid);
      Point3d l2 = new Point3d (Lcentroid);
      Vector3d extL1 = new Vector3d (Ldir);
      extL1.normalize ().scale (2);
      l1.add (extL1);
      l2.sub (extL1);
      Point3d[] PFTL = meshHelper.pointsForTransform (l1, l2, Lnorm);

      l1 = new Point3d (centreGuide);
      l2 = new Point3d (centreGuide);
      extL = new Vector3d (slotDir);
      extL.normalize ().scale (2);
      l1.add (extL);
      l2.sub (extL);
      Point3d[] PFTG =
         meshHelper
            .pointsForTransform (
               l1, l2, new Vector3d (root.fromCenterToSurface));

      RigidTransform3d transformL = meshHelper.SVDRegistration (PFTG, PFTL);
      L.transform (transformL);

      Assist.GetMesh (mechModel, "ClippedDonor").computeCentroid (centreGuide);
      RigidTransform3d posec =
         Assist.GetRigidBody (mechModel, "ClippedDonor").getPose ();
      centreGuide.transform (posec);
      OBB FBBB = newBase.computeOBB ();
      Vector3d FBWidths = new Vector3d ();
      FBBB.getWidths (FBWidths);
      double maxW = 0.0;
      for (int i = 0; i < 3; i++) {
         if (FBWidths.get (i) > maxW) {
            maxW = FBWidths.get (i);
         }
      }

      Vector3d direction = new Vector3d (fibDir).negate ();
      direction.normalize ();
      direction.scale (height / 2 - 2);
      Point3d position = new Point3d (centreGuide);
      position.add (direction);
      position
         .add (
            new Vector3d (
               root.fromCenterToSurface.normalize ().scale (radius - 2)));
      meshHelper.setPlaneOrigin (L, position);

      // Differencing marking from donorGuide
      SurfaceMeshIntersector marking = new SurfaceMeshIntersector ();
      DonorGuide = marking.findUnion (DonorGuide, L);

      // Creating slots
      for (int i = 0; i < (2 * root.numberOfSegments); i++) {
         // Computing Centroid of Cutting Planes
         Point3d centerplane = new Point3d ();
         root.cuttingPlanes.get (i).computeCentroid (centerplane);
         RigidTransform3d pose =
            root.cuttingPlanesMeshBodies.get (i).getPose ();
         centerplane.transform (pose);

         String homedir = ArtisynthPath.getHomeDir ();
         File pathHome = new File (homedir);
         String homeParent = pathHome.getParentFile ().getAbsolutePath ();
         if (root.DEBUG) {
            System.out.println ("Parent Directory: " + homeParent);
         }

         String path =
            homeParent
            + "/artisynth_projects/src/artisynth/models/Prisman/stl/squareFrame.stl";
         PolygonalMesh squareframelong =
            meshHelper.readMesh (path, "squareFrame.stl");

         // Creating Boxes for slots
         PolygonalMesh Largebox = new PolygonalMesh (squareframelong.clone ());
         path =
            homeParent
            + "/artisynth_projects/src/artisynth/models/Prisman/stl/bladeGuideUnion.stl";
         PolygonalMesh metalinsertlong =
            meshHelper.readMesh (path, "bladeGuideUnion.stl");
         PolygonalMesh Smallbox = new PolygonalMesh (metalinsertlong.clone ());
         large.add (i, Largebox.clone ());
         small.add (i, Smallbox.clone ());

         // Adding FixedMeshBodies to mechModel
         largeBodies
            .add (
               i,
               new FixedMeshBody ("Large" + String.valueOf (i), large.get (i)));
         smallBodies
            .add (
               i,
               new FixedMeshBody ("Small" + String.valueOf (i), small.get (i)));
      }

      Point3d[] centerIntersections = new Point3d[2 * root.numberOfSegments];
      Point3d firstcenter = new Point3d ();

      for (int i = 0; i < (2 * root.numberOfSegments); i++) {
         // Getting Cutting Planes' Normal Vectors and Centroid
         Vector3d normal = root.cuttingPlanes.get (i).getNormal (0);
         Point3d centerplane = new Point3d ();
         root.cuttingPlanes.get (i).computeCentroid (centerplane);
         RigidTransform3d pose =
            root.cuttingPlanesMeshBodies.get (i).getPose ();
         normal.transform (pose);
         centerplane.transform (pose);

         // Getting Largeboxes' Normal Vectors and Centroid
         OBB largebb = large.get (i).computeOBB ();
         Vector3d[] axeslarge = new Vector3d[3];
         for (int j = 0; j < axeslarge.length; j++) {
            axeslarge[j] = new Vector3d ();
         }
         largebb.getSortedAxes (axeslarge);

         RigidTransform3d poselarge = largeBodies.get (i).getPose ();
         Vector3d largeDir = new Vector3d (axeslarge[0].copy ());
         Vector3d largenormal = new Vector3d (axeslarge[1].copy ());
         largenormal.transform (poselarge);
         largeDir.transform (poselarge);
         Point3d largecenter = new Point3d ();
         large.get (i).computeCentroid (largecenter);
         largecenter.transform (poselarge);

         Vector3d extension =
            mathHelper
               .ProjectLineToPlane (
                  new Vector3d (slotDir), new Vector3d (normal));
         extension.scale (40);
         Vector3d tosideExt = new Vector3d (root.fromCenterToSurface);
         if (!root.donorIsScapulaCheckBox.isSelected ()) {
            tosideExt =
               mathHelper
                  .ProjectLineToPlane (
                     new Vector3d (tosideExt), new Vector3d (normal));
            tosideExt.normalize ();
            double costheta =
               tosideExt.dot (root.fromCenterToSurface)
               / (tosideExt.norm () * root.fromCenterToSurface.norm ());
            double scalefactor = (radius - 4) / costheta;
            tosideExt.scale (scalefactor);
         }
         // Recalculating to side for scapula donor bone
         else {
            Plane plane = new Plane ();
            RigidTransform3d setting = new RigidTransform3d ();
            RotationMatrix3d rotate =
               meshHelper
                  .rotatePlane (
                     new Vector3d (0, 0, 1),
                     new Vector3d (root.cuttingPlanes.get (i).getNormal (0)));
            setting.setRotation (rotate);
            Vector3d centroidPlane = new Vector3d ();
            root.cuttingPlanes.get (i).computeCentroid (centroidPlane);
            setting.setTranslation (centroidPlane);
            plane.set (setting);

            // Finding center of intersection between cuttingPlane[i] and
            // clippedDonor
            BVIntersector contourintersector = new BVIntersector ();
            ArrayList<LinkedList<Point3d>> contour =
               new ArrayList<LinkedList<Point3d>> ();
            contour =
               contourintersector
                  .intersectMeshPlane (
                     Assist.GetMesh (mechModel, "ClippedDonor").clone (), plane,
                     0.01);
            ArrayList<PolygonalMesh> contourPlane =
               new ArrayList<PolygonalMesh> ();
            for (int h = 0; h < contour.size (); h++) {
               contourPlane.add (h, new PolygonalMesh ());
               for (int g = 0; g < contour.get (h).size (); g++) {
                  Vertex3d newVert = new Vertex3d (contour.get (h).get (g));
                  contourPlane.get (h).addVertex (newVert);
               }
            }
            PolygonalMesh chosenContour = new PolygonalMesh ();
            FixedMeshBody chosenContourBody = new FixedMeshBody ();
            double maxarea = 0;
            for (int h = 0; h < contour.size (); h++) {
               Point3d contourcentroid = new Point3d ();
               contourPlane.get (h).computeCentroid (contourcentroid);
               FixedMeshBody contourIntersection =
                  new FixedMeshBody ("contour1", contourPlane.get (h));
               RigidTransform3d X = contourIntersection.getPose ();
               OBB contourOBB = contourPlane.get (h).computeOBB ();
               Vector3d cw = new Vector3d ();
               contourOBB.getWidths (cw);
               double area = meshHelper.ComputeContourArea (cw);
               if (area > maxarea) {
                  maxarea = area;
                  chosenContour = new PolygonalMesh (contourPlane.get (h));
                  chosenContourBody =
                     new FixedMeshBody ("Contour1", chosenContour);
               }
            }

            Point3d center = new Point3d ();
            chosenContour.computeCentroid (center);
            RigidTransform3d X = chosenContourBody.getPose ();
            center.transform (X);
            centerplane = new Point3d (center);
            centerIntersections[i] = new Point3d (center);

            Vector3d[] tAxis = new Vector3d[3];
            for (int k = 0; k < 3; k++) {
               tAxis[k] = new Vector3d ();
            }
            OBB trimBB =
               Assist.GetMesh (mechModel, "ScapulaTrimPlane").computeOBB ();
            trimBB.getSortedAxes (tAxis);
            Vector3d extSide = new Vector3d (tAxis[1]);
            extSide
               .transform (
                  Assist
                     .GetMeshBody (mechModel, "ScapulaTrimPlane").getPose ());
            if (extSide.dot (root.fromCenterToSurface) < 0) {
               extSide.negate ();
            }

            Vector3d lengthDir = new Vector3d (tAxis[0]);
            lengthDir
               .transform (
                  Assist
                     .GetMeshBody (mechModel, "ScapulaTrimPlane").getPose ());
            lengthDir.normalize ();

            tosideExt =
               mathHelper
                  .ProjectLineToPlane (
                     new Vector3d (extSide), new Vector3d (normal));
            tosideExt.normalize ();
            double costheta =
               tosideExt.dot (extSide) / (tosideExt.norm () * extSide.norm ());

            if (i == 0) {
               double scalefactor = 16.0 / costheta;
               tosideExt.scale (scalefactor);
               firstcenter = new Point3d (centerplane);
            }
            else {
               double scalefactor = 16.0 / costheta;
               tosideExt.scale (scalefactor);
               centerplane =
                  new Point3d (
                     mathHelper
                        .intersectLinePlane (
                           new Point3d (firstcenter), new Vector3d (lengthDir),
                           new Point3d (centerIntersections[i]),
                           new Vector3d (normal)));
            }
         }

         Point3d point1 = new Point3d (centerplane);
         point1.add (extension);
         point1.add (tosideExt);
         Point3d point2 = new Point3d (centerplane);
         point2.sub (extension);
         point2.add (tosideExt);
         Vector3d normalPFTPlane =
            new Vector3d (normal.get (0), normal.get (1), normal.get (2));
         // what the lmao
         if (i % 2 != 0) {
            normalPFTPlane.negate ();
         }
         Point3d[] PFTPlane =
            meshHelper.pointsForTransform (point1, point2, normalPFTPlane);

         point1 = new Point3d (largecenter);
         point2 = new Point3d (largecenter);
         extension = new Vector3d (largeDir);
         extension.scale (40);
         point1.add (extension);
         point2.sub (extension);
         Point3d[] PFTLarge =
            meshHelper.pointsForTransform (point1, point2, largenormal);

         RigidTransform3d transformLarge =
            meshHelper.SVDRegistration (PFTPlane, PFTLarge);
         large.get (i).transform (transformLarge);
         small.get (i).transform (transformLarge);
      }

      Point3d pTop = new Point3d ();
      root.cuttingPlanes.get (0).computeCentroid (pTop);
      Point3d pBottom = new Point3d ();
      root.cuttingPlanes
         .get (2 * root.numberOfSegments - 1).computeCentroid (pBottom);

      double lengthActual = pTop.distance (pBottom);
      Point3d pMid = new Point3d (pTop);
      pMid.add (pBottom);
      pMid.scale (0.5);

      // Realigning base when donor is scapula
      if (root.donorIsScapulaCheckBox.isSelected ()) {
         Base =
            MeshFactory.createBox (lengthActual + 20, 12, 12, pMid, 35, 8, 4);
         BaseBody = new FixedMeshBody ("Base", Base);

         basePose = BaseBody.getPose ();

         basebb = Base.computeOBB ();
         basebb.getSortedAxes (axis);
         longest = new Vector3d (axis[0].copy ());
         normalbase = new Vector3d (axis[1].copy ());
         basecenter = new Point3d ();
         basebb.getCenter (basecenter);
         longest.transform (basePose);
         longest.normalize ();
         normalbase.transform (basePose);
         normalbase.normalize ();
         basecenter.transform (basePose);

         p1 = new Point3d (basecenter);
         p2 = new Point3d (basecenter);
         ext = new Vector3d (longest);
         ext.scale (length);
         p1.add (ext);
         p2.sub (ext);
         PFTBase =
            meshHelper
               .pointsForTransform (p1, p2, new Vector3d (normalbase.copy ()));

         OBB trimBB =
            Assist.GetMesh (mechModel, "ScapulaTrimPlane").computeOBB ();
         RigidTransform3d poseTrim =
            Assist.GetMeshBody (mechModel, "ScapulaTrimPlane").getPose ();

         Vector3d[] tAxis = new Vector3d[3];
         for (int i = 0; i < 3; i++) {
            tAxis[i] = new Vector3d ();
         }
         trimBB.getSortedAxes (tAxis);
         Vector3d newBaseLengthDir = new Vector3d (tAxis[0]);
         newBaseLengthDir.transform (poseTrim);
         newBaseLengthDir.normalize ();

         Vector3d trimPlaneNormal =
            new Vector3d (
               Assist.GetMesh (mechModel, "ScapulaTrimPlane").getNormal (0));
         trimPlaneNormal.transform (poseTrim);

         Vector3d topBottom = new Vector3d (pBottom);
         topBottom.sub (pTop);
         topBottom.normalize ();

         if (newBaseLengthDir.dot (topBottom) < 0) {
            newBaseLengthDir.negate ();
         }

         Vector3d newBaseSideDir = new Vector3d (tAxis[1]);
         newBaseSideDir.transform (poseTrim);

         Plane plane = new Plane ();
         RigidTransform3d poseTrimPlane =
            Assist.GetMeshBody (mechModel, "ScapulaTrimPlane").getPose ();
         plane.set (poseTrimPlane);

         // Finding center of intersection between scapulaTrimPlane and
         // originalDonor
         BVIntersector contourintersector = new BVIntersector ();
         ArrayList<LinkedList<Point3d>> contour =
            new ArrayList<LinkedList<Point3d>> ();
         contour =
            contourintersector
               .intersectMeshPlane (
                  Assist.GetMesh (mechModel, "Donor").clone (), plane, 0.01);
         ArrayList<PolygonalMesh> contourPlane =
            new ArrayList<PolygonalMesh> ();
         for (int h = 0; h < contour.size (); h++) {
            contourPlane.add (h, new PolygonalMesh ());
            for (int g = 0; g < contour.get (h).size (); g++) {
               Vertex3d newVert = new Vertex3d (contour.get (h).get (g));
               contourPlane.get (h).addVertex (newVert);
            }
         }
         PolygonalMesh chosenContour = new PolygonalMesh ();
         FixedMeshBody chosenContourBody = new FixedMeshBody ();
         double maxarea = 0;
         for (int h = 0; h < contour.size (); h++) {
            Point3d contourcentroid = new Point3d ();
            contourPlane.get (h).computeCentroid (contourcentroid);
            FixedMeshBody contourIntersection =
               new FixedMeshBody ("contour1", contourPlane.get (h));
            RigidTransform3d X = contourIntersection.getPose ();
            OBB contourOBB = contourPlane.get (h).computeOBB ();
            Vector3d cw = new Vector3d ();
            contourOBB.getWidths (cw);
            double area = meshHelper.ComputeContourArea (cw);
            if (area > maxarea) {
               maxarea = area;
               chosenContour = new PolygonalMesh (contourPlane.get (h));
               chosenContourBody =
                  new FixedMeshBody ("Contour1", chosenContour);
            }
         }

         Point3d center = new Point3d ();
         chosenContour.computeCentroid (center);
         RigidTransform3d X = chosenContourBody.getPose ();
         center.transform (X);

         Vector3d tPoint = new Vector3d (center);
         Vector3d bPoint = new Vector3d (center);
         Vector3d centerExt = new Vector3d (newBaseLengthDir);
         centerExt.normalize ().scale (50);
         tPoint.sub (centerExt);
         bPoint.add (centerExt);

         VectorNd mB =
            mathHelper
               .projectPointToLine (
                  new Point3d (pMid), new Vector3d (tPoint),
                  new Vector3d (bPoint));
         Vector3d midBase = new Vector3d (mB.get (0), mB.get (1), mB.get (2));
         Vector3d extToMatchPlane = new Vector3d (trimPlaneNormal);
         extToMatchPlane.negate ();
         extToMatchPlane.scale (6);
         midBase.add (extToMatchPlane);

         pTop = new Point3d ();
         large.get (0).computeCentroid (pTop);
         pBottom = new Point3d ();
         large.get (2 * root.numberOfSegments - 1).computeCentroid (pBottom);
         Point3d midGuide = new Point3d (pTop);
         midGuide.add (pBottom);
         midGuide.scale (0.5);
         double scalingSide = midBase.distance (midGuide);

         Point3d test1 = new Point3d (midBase);
         Point3d test2 = new Point3d (midBase);
         Vector3d extTest = new Vector3d (newBaseSideDir);
         extTest.normalize ().scale (scalingSide);
         test1.add (extTest);
         test2.sub (extTest);

         if (test1.distance (midGuide) > test2.distance (midGuide)) {
            newBaseSideDir.negate ();
         }

         p1 = new Point3d (midBase);
         p2 = new Point3d (midBase);
         toside = new Vector3d (newBaseSideDir);
         toside.normalize ();
         toside.scale (scalingSide);
         p1.add (toside);
         p2.add (toside);
         ext = new Vector3d (newBaseLengthDir);
         ext.normalize ();
         ext.scale (length);
         p1.add (ext);
         p2.sub (ext);
         PFTFib =
            meshHelper
               .pointsForTransform (p1, p2, new Vector3d (newBaseSideDir));

         tBase = meshHelper.SVDRegistration (PFTFib, PFTBase);
         Base.transform (tBase);
         DonorGuide = new PolygonalMesh (Base.clone ());

      }

      // Getting Union of base and large boxes
      for (int i = 0; i < (2 * root.numberOfSegments); i++) {
         DonorGuide = MeshFactory.getUnion (DonorGuide, large.get (i));
      }

      // Removing parts of the DonorGuide that intersects with smallBoxes
      SurfaceMeshIntersector intersector1 = new SurfaceMeshIntersector ();

      for (int i = 0; i < (2 * root.numberOfSegments); i++) {
         DonorGuide = intersector1.findDifference01 (DonorGuide, small.get (i));
      }

      // Printing Plane Info For Capstone Group
      try {
         PrintWriter out = new PrintWriter ("out.txt", "UTF-8");
         out.println ("Diameter:" + String.valueOf (radius * 2));
         out
            .println (
               "Distance from center of Donor to edge of guide: "
               + String.valueOf (radius + 6));
         Point3d top = new Point3d (root.topPoint);
         top
            .add (
               new Vector3d (root.fromCenterToSurface)
                  .normalize ().scale (radius));
         for (int i = 0; i < (2 * root.numberOfSegments); i++) {
            out
               .println (
                  "Printing Information for Cutting Plane: "
                  + String.valueOf (i));
            Vector3d normal =
               new Vector3d (root.cuttingPlanes.get (i).getNormal (0));
            Point3d center = new Point3d ();
            root.cuttingPlanes.get (i).computeCentroid (center);
            Point3d centerplane = new Point3d (center);
            RigidTransform3d pose =
               root.cuttingPlanesMeshBodies.get (i).getPose ();
            normal.transform (pose);
            Vector3d tosideExt = new Vector3d (root.fromCenterToSurface);
            tosideExt =
               mathHelper.ProjectLineToPlane (tosideExt, new Vector3d (normal));
            tosideExt.normalize ();
            double costheta =
               tosideExt.dot (root.fromCenterToSurface)
               / (tosideExt.norm () * root.fromCenterToSurface.norm ());
            double scalefactor = (radius) / costheta;
            tosideExt.scale (scalefactor);
            if ((i % 2) == 0) {
               normal.negate ();
            }
            centerplane.transform (pose);
            centerplane.add (tosideExt);
            out
               .println (
                  "Position: " + String.valueOf (top.distance (centerplane)));
            out
               .println (
                  "Angle X: " + String
                     .valueOf (
                        mathHelper
                           .calculateAngle (normal, new Vector3d (slotDir))
                        * (-1)));
            out
               .println (
                  "Angle Y: " + String
                     .valueOf (
                        mathHelper
                           .calculateAngle (
                              normal,
                              new Vector3d (root.fromCenterToSurface))));
         }
         out.close ();
      }
      catch (IOException e) {
         e.printStackTrace ();
      }

      BaseBody.getRenderProps ().setVisible (false);
      List<PolygonalMesh> screwsonsegment = new ArrayList<PolygonalMesh> ();

      PolygonalMesh reconstructedWithDonorGuideHoles =
         new PolygonalMesh (
            Assist.GetMesh (mechModel, "Reconstructed"+Target).clone ());
      FixedMeshBody reconstructedWithDonorGuideHolesBody =
         new FixedMeshBody (
            "Reconstructed Maxilla With Donor Guide Holes",
            reconstructedWithDonorGuideHoles);
      // Adding screwholes and donor guide marking when donor is fibula
      if (!root.donorIsScapulaCheckBox.isSelected ()) {
         PolygonalMesh tempSegmentMesh = new PolygonalMesh ();
         tempSegmentMesh = null;

         reconstructedWithDonorGuideHolesBody =
            new FixedMeshBody (
               "Reconstructed Maxilla With Donor Guide Holes",
               reconstructedWithDonorGuideHoles);

         DonorGuide = marking.findDifference01 (DonorGuide, cylinder);
      }
      FixedMeshBody DonorGuideBody =
         new FixedMeshBody ("Donor Guide", DonorGuide);

      // reconstructedWithDonorGuideHoles =
      // new PolygonalMesh (mandiblereconstructed.clone ());
      // reconstructedWithDonorGuideHolesBody =
      // new FixedMeshBody (
      // "Reconstructed Maxilla With Donor Guide Holes",
      // reconstructedWithDonorGuideHoles);

      // if (!donorIsScapulaCheckBox.isSelected ()) {
      // SurfaceMeshIntersector in = new SurfaceMeshIntersector ();
      // for (int i = 0; i < screwsonsegment.size (); i++) {
      // reconstructedWithDonorGuideHoles =
      // in.findDifference01 (
      // reconstructedWithDonorGuideHoles.clone (),
      // screwsonsegment.get (i).clone ());
      // }
      // }
      for (MeshComponent mech: mechModel.meshBodies ()) {
         if (mech.getName ()!=null && (mech.getName ().contains ("Small") || mech.getName ().contains ("Large"))) {
            mechModel.removeMeshBody (mech);
         }
      }
      mechModel.addMeshBody (cbody);
      cbody.getRenderProps ().setVisible (false);
      mechModel.addMeshBody (BaseBody);
      BaseBody.getRenderProps ().setVisible (false);
      mechModel.addMeshBody (DonorGuideBody);
      if (!root.donorIsScapulaCheckBox.isSelected ()) {
         Assist.GetMeshBody (mechModel, "Reconstructed"+Target).getRenderProps ().setVisible (false);
         mechModel.addMeshBody (reconstructedWithDonorGuideHolesBody);
      }
      mechModel.removeMeshBody (cbody);
      root.getMainViewer ().autoFit ();
      root.log.append("Finished creating Donor Guide\n");
   }

   @Override
   public void undo () {
      mechModel.removeMeshBody (Assist.GetMeshBody (mechModel, "Cylinder"));
      mechModel.removeMeshBody (Assist.GetMeshBody (mechModel, "Base"));
      mechModel.removeMeshBody (Assist.GetMeshBody (mechModel, "Donor Guide"));
      mechModel.removeMeshBody(Assist.GetMeshBody (mechModel, "Reconstructed Maxilla With Donor Guide Holes"));
   }

   @Override
   public String getName () {
      return("Fibula Guide");
   }
}