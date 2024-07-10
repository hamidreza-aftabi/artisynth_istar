package artisynth.istar.MandibleRecon;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.swing.AbstractAction;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.AnisotropicLinearMaterial;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ArtisynthPath;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMathFunctions;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.MeshICP;
import maspack.geometry.OBB;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Polyline;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.Interpolation;
import maspack.interpolation.Interpolation.Order;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer.LineStyle;

public class plateDeformPrepClicked extends AbstractAction {
   ReconstructionModel root;
   MechModel mechModel;
   Assist Assist = new Assist ();
   ArrayList<Polyline> lines;
   FemModel3d fem;
   Vector3d BrickMeshWidths;
   Integer plateBendSegments = 20;
   Point3d plateStart;
   int SHIFT_IDX = 100;
   FixedMeshBody newrdpMeshBody;
   List<FrameMarker> plateBendFrameMarkers = new ArrayList<FrameMarker> ();
   public String meshDirectory =
      ArtisynthPath.getSrcRelativePath (this, "FemTest\\");
   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   FixedMeshBody[] screwBodies = new FixedMeshBody[17];
   PolygonalMesh[] screwMeshes = new PolygonalMesh[17];
   String Target;
   
   //CHANGE LINE 428 IF PLATE SHOWING UP IN WRONG DIRECTION

   public plateDeformPrepClicked (ReconstructionModel root, MechModel mechModel,
   String Target) {
      putValue (NAME, "Prep Plate Deform");
      this.root = root;
      this.mechModel = mechModel;
      this.Target = Target;
   }

   @Override
   public void actionPerformed (ActionEvent evt) {
      RigidBody reconBody = new RigidBody ("Recon");
      // This is the default
      reconBody
         .setSurfaceMesh (Assist.GetMesh (mechModel, "Reconstructed" + Target));
      // this one is for scapula mandible lmao
      // reconBody.setMesh(maxillaMesh);
      mechModel.addRigidBody (reconBody);

      reconBody.setDynamic (Boolean.FALSE);
      Assist.GetRigidBody (mechModel, Target).setDynamic (Boolean.FALSE);
      PolygonalMesh brickMesh =
         Assist.loadGeometry (meshDirectory, "MasterSquarePrism.stl");
      PolygonalMesh actualPlateMesh =
         Assist.loadGeometry (meshDirectory, "MasterPlateSquare.stl");
      OBB boundingBox = new OBB (brickMesh);

      BrickMeshWidths = new Vector3d ();
      boundingBox.getWidths (BrickMeshWidths);

      // Fed Model Creation

      // FemFactory.createFromMesh(fem, brickMesh, 1 );
      fem = new FemModel3d ("fem");

      FemFactory
         .createHexGrid (
            fem, BrickMeshWidths.get (0), BrickMeshWidths.get (1),
            BrickMeshWidths.get (2), /* nx= */plateBendSegments, /* ny= */1,
            /* nz= */1);
      double nodeLength = BrickMeshWidths.get (0) / plateBendSegments;

      AnisotropicLinearMaterial material2 = new AnisotropicLinearMaterial ();
      double[] diag = { 100, 2, 3, 4, 5, 6, 7, 8, 9 };
      // why is this the stiffnesstensor?
      Matrix6d stiffTensor2 = material2.getStiffnessTensor ();
      stiffTensor2.scale (2);
      stiffTensor2.set (0, 0, 0.9 * stiffTensor2.m00);

      material2.setStiffnessTensor (stiffTensor2);
      fem.setMaterial (material2);
      fem.setDensity (1000);
      fem.setStiffnessDamping (10);
      fem.setParticleDamping (0.1);
      mechModel.addModel (fem);

      lines =
         ((PolylineMesh)Assist.GetNonPolyMesh (mechModel, "Plate")).getLines ();
      System.out.println ("The lines size is " + lines.size ());
      
      //get round plate distance- just adding vectors between framemarkers
      //trim using plane normal (-1,0,0)
      //need a point to define plane: (length of round plate, 0, 0)
      FrameMarker[] frameMarkers = 
         Assist.GetRigidBody (mechModel, "Mandible").getFrameMarkers ();
      int sizeframes = frameMarkers.length - 6;
      double length = 0.0;
      
      for (int i = 0; i < sizeframes - 1; i++) {
         Point3d p1 = frameMarkers[i].getPosition ();
         Point3d p2 = frameMarkers[i + 1].getPosition ();
         
         Vector3d vec = p1.sub (p2);
         length = length + vec.norm ();
      }
      
      Point3d definePlane = new Point3d (length, 0, 0);
      Vector3d trimNormal = new Vector3d (-1, 0, 0);
      
      PolygonalMesh trimPlane = MeshFactory.createPlane (90, 90);
      
      meshHelper.createPlane (new Vector3d (0,0,0), trimNormal, definePlane, null, trimPlane);
      
      SurfaceMeshIntersector intersector = new SurfaceMeshIntersector ();

      intersector.findDifference01 (actualPlateMesh, trimPlane);
      //finished trimming plate
      
      Point3d c1 = new Point3d ();
      Point3d c2 = new Point3d ();
      Vector3d planenormal1;
      RigidTransform3d pose;
      Point3d centroid1;
      Vector3d planenormal2;
      Point3d centroid2;
      if (Target.equals ("Maxilla")) {
         root.activePlaneList.get (0).getMesh ().computeCentroid (c1);
         planenormal1 =
            new Vector3d (
               root.activePlaneList.get (0).getMesh ().getNormal (0));
         pose = root.activePlaneList.get (0).getPose ();
         centroid1 = new Point3d (c1);

         root.activePlaneList.get (1).getMesh ().computeCentroid (c2);
         planenormal2 =
            new Vector3d (
               root.activePlaneList.get (1).getMesh ().getNormal (0));
         pose = root.activePlaneList.get (1).getPose ();
         centroid2 = new Point3d (c2);
         centroid2.transform (pose);
         planenormal2.transform (pose);
      }
      else {
         Assist.GetMesh (mechModel, "Plane1").computeCentroid (c1);
         planenormal1 =
            new Vector3d (Assist.GetMesh (mechModel, "Plane1").getNormal (0));
         pose = Assist.GetMeshBody (mechModel, "Plane1").getPose ();
         centroid1 = new Point3d (c1);

         Assist.GetMesh (mechModel, "Plane2").computeCentroid (c2);
         planenormal2 =
            new Vector3d (Assist.GetMesh (mechModel, "Plane2").getNormal (0));
         pose = Assist.GetMeshBody (mechModel, "Plane2").getPose ();
         centroid2 = new Point3d (c2);
         centroid2.transform (pose);
         planenormal2.transform (pose);
      }
      centroid1.transform (pose);
      planenormal1.transform (pose);

      // if (setPlaneFile != null) {
      // planenormal1 = new Vector3d (normalPlane1);
      // planenormal2 = new Vector3d (normalPlane2);
      // centroid1 = new Point3d (centerPlane1);
      // centroid2 = new Point3d (centerPlane2);
      //
      // planenormal1.transform (posetr1);
      // planenormal2.transform (posetr2);
      // centroid1.transform (posetr1);
      // centroid2.transform (posetr2);
      // }

      // Creating a reference to the plateNumericList depending on if
      // implants is checked
      NumericList generalNumericList = new NumericList (3);
      generalNumericList = root.plateNumericList; // All references to
      // plateNumericList replaced
      // with general NumericList

      NumericListKnot knot1 =
         mathHelper
            .closestNumericListKnotToPlane (
               planenormal1, centroid1, generalNumericList);

      NumericListKnot knot2 =
         mathHelper
            .closestNumericListKnotToPlane (
               planenormal2, centroid2, generalNumericList);

      // Creating new numericList to only contain knots between the bounds
      // Extending the knots be 1 on eachside so that the polyline is a
      // little past the plane, for csg
      NumericListKnot lowestKnot = new NumericListKnot (knot1);
      NumericListKnot highestKnot = new NumericListKnot (knot2);
      
      
      if (!knot1.getPrev ().equals (null)) {
         lowestKnot = new NumericListKnot (knot1.getPrev ());
      } 
      
      if (!knot2.getNext ().equals (null)) {
         highestKnot = new NumericListKnot (knot2.getNext ());
      } 
     // NumericListKnot lowestKnot = new NumericListKNot (knot1.getPrev()); 
     // NumericListKnot highestKnot = new NumericListKnot (knot2.getNext ());
      if (knot2.t < lowestKnot.t) {
         lowestKnot = new NumericListKnot (knot2.getPrev ());
         highestKnot = new NumericListKnot (knot1.getNext ());
      }

      // Only knots between the bounds are copied to curatedList
      NumericList curatedListLower = new NumericList (3);
      NumericList curatedListHigher = new NumericList (3);
      NumericList curatedList = new NumericList (3);
      Iterator<NumericListKnot> itr = generalNumericList.iterator ();
      NumericListKnot tempKnot;
      int knotsAdded = 0;
      while (itr.hasNext ()) {
         tempKnot = new NumericListKnot (itr.next ());
         if (tempKnot.t < lowestKnot.t) {
            curatedListLower.add (new NumericListKnot (tempKnot));
            knotsAdded++;
         }
         else if (tempKnot.t > highestKnot.t) {
            curatedListHigher.add (new NumericListKnot (tempKnot));
            knotsAdded++;
         }
      }

      for (NumericListKnot knot : curatedListLower) {
         curatedList.add (knot);
      }

      int numberOfSubdivisions = 5000 - knotsAdded - 1;
      int size = root.simpList.getNumKnots ();
      System.out.println (size);
      NumericList newplateNumericList = new NumericList (3);
      Interpolation linear = new Interpolation ();
      linear.setOrder (Order.Linear);
      newplateNumericList.setInterpolation (linear);

      Point3d[] points = new Point3d[size];

      int idx = 0;
      for (NumericListKnot knot : root.simpList) {
         if (root.DEBUG) {
            System.out.println ("knot vector is " + knot.v);
            System.out.println ("knot time is " + knot.t);
         }
         Point3d knotPos = new Point3d ();
         knotPos.x = knot.v.get (0);
         knotPos.y = knot.v.get (1);
         knotPos.z = knot.v.get (2);
         points[idx] = knotPos;
         newplateNumericList.add (points[idx], knot.t);
         idx++;

      }
      System.out.println (newplateNumericList);

      // for (int i = 0; i < size; i++) {
      // points[i] = frameMarkers[i].getPosition ();
      // newplateNumericList
      // .add (points[i], i * numberOfSubdivisions / (size - 1));
      //
      // }

      VectorNd[] interpolatedVectors = new VectorNd[numberOfSubdivisions];

      for (int i = 0; i < numberOfSubdivisions; i++) {
         interpolatedVectors[i] = new VectorNd ();
         interpolatedVectors[i].setSize (3);
         newplateNumericList
            .interpolate (
               interpolatedVectors[i], newplateNumericList.getFirst ().t + i);

      }

      // Adding Results of interpolation to the numericList
      for (int i = 0; i < numberOfSubdivisions; i++) {
         newplateNumericList
            .add (
               interpolatedVectors[i], newplateNumericList.getFirst ().t + i);
      }

      for (NumericListKnot knot : newplateNumericList) {
         curatedList.add (knot);
      }

      for (NumericListKnot knot : curatedListHigher) {
         curatedList.add (knot);
      }

      int sizeCurList = curatedList.getNumKnots ();
      int numberOfKnots = sizeCurList - 1;
      if (root.DEBUG) {
         System.out.println ("knots: " + sizeCurList);
      }

      Point3d[] curatedPoints = new Point3d[sizeCurList];
      int[][] indices = new int[numberOfKnots][2];

      Iterator<NumericListKnot> simpItr = curatedList.iterator ();
      int j = 0;
      while (simpItr.hasNext ()) {
         curatedPoints[j] = new Point3d (simpItr.next ().v);
         j++;
      }

      for (j = 0; j < numberOfKnots; j++) {
         indices[j][0] = j;
         indices[j][1] = j + 1;
      }
      PolylineMesh newsphericalPolyLine =
         MeshFactory.createSphericalPolyline (50, 50, 50);
      if (root.DEBUG) {
         System.out.println ("cur points: " + curatedPoints.length);
         System.out.println ("indices: " + indices.length);
      }

      PolylineMesh newrdpMesh = new PolylineMesh ();
      newrdpMesh.addMesh (newsphericalPolyLine);
      newrdpMesh.set (curatedPoints, indices);
      newrdpMeshBody = new FixedMeshBody ("New Clipped Plate", newrdpMesh);
      mechModel.add (newrdpMeshBody);

      RenderProps.setLineStyle (newrdpMeshBody, LineStyle.CYLINDER);

      lines = newrdpMesh.getLines ();

      // Initial Positioning:
      // Plate
      plateStart = new Point3d ();
      Point3d center = Assist.GetRigidBody (mechModel, Target).getPosition ();
      // double distancetocenter = Integer.MAX_VALUE;
      // for (Polyline point : lines) {
      // if (Math
      // .abs (
      // center.z
      // - point.getVertex (0).getWorldPoint ().z) < distancetocenter) {
      // distancetocenter =
      // Math.abs (center.z - point.getVertex (0).getWorldPoint ().z);
      // plateStart = point.getVertex (0).getWorldPoint ();
      // }
      // }

      Vector3d plateInitialDir = new Vector3d ();
      // lines.get (0).getSegments()[0].getDirection (plateInitialDir);
      // plateInitialDir.sub (lines.get (100).getVertex(0).getWorldPoint (),
      // lines.get (0).getVertex(0).getWorldPoint ());

      plateInitialDir
         .sub (
            lines
               .get ((Integer)(lines.size () / 2) - 1).getVertex (0)
               .getWorldPoint (),
            lines
               .get ((Integer)(lines.size () / 2) + 1).getVertex (0)
               .getWorldPoint ());
      // plateInitialDir.normalize ();
      // plateInitialDir.scale (10);

      plateStart =
         lines.get ((lines.size () - 1) / 2).getVertex (0).getWorldPoint ();

      Vector3d plateSecondPoint = new Vector3d (plateStart);
      plateSecondPoint.add (plateInitialDir);

      int numV = Assist.GetMesh (mechModel, Target).numVertices ();
      double mindistance = 1.0;
      // double mindistance = 100.0;
      double distance = 0.0;
      int idx1 = 0;

      for (int i = 0; i < numV; i++) {
         Vertex3d v = Assist.GetMesh (mechModel, Target).getVertex (i);
         distance = v.distance (plateSecondPoint);
         if (distance < mindistance) {
            mindistance = distance;
            idx1 = i;
         }
      }

      Vector3d plateNormal = new Vector3d ();
      Vertex3d vertex1 = Assist.GetMesh (mechModel, Target).getVertex (idx1);
      vertex1.computeNormal (plateNormal);
      //plateNormal.negate ();

      HelperMeshFunctions meshHelper = new HelperMeshFunctions ();

//       plateNormal.negate ();
//       plateSecondPoint.negate ();

      Point3d[] platePointsForTransform;
      platePointsForTransform =
         meshHelper
            .pointsForTransform3 (
               plateNormal, new Vector3d (plateStart),
               new Vector3d (plateSecondPoint), Boolean.TRUE);

      Vector3d brickCentroid = new Vector3d ();
      brickMesh.computeCentroid (brickCentroid);

      Point3d brickStart = new Point3d (brickCentroid);
      Point3d brickSecondPoint = new Point3d (brickCentroid);
      Vector3d brickDir = new Vector3d (1, 0, 0);
      Vector3d brickNormal = new Vector3d (0, 0, 1);

      brickStart.add (brickDir);
      brickSecondPoint.sub (brickDir);

      Point3d[] brickPointsForTransform = new Point3d[3];
      brickPointsForTransform =
         meshHelper
            .pointsForTransform3 (
               brickNormal, new Vector3d (brickStart),
               new Vector3d (brickSecondPoint), Boolean.FALSE);

      RigidTransform3d initTransform =
         meshHelper
            .SVDRegistration (platePointsForTransform, brickPointsForTransform);
      // meshHelper
      // .SVDRegistration (platePointsForTransform, brickPointsForTransform);
      brickMesh.transform (initTransform);
      actualPlateMesh.transform (initTransform);

      FixedMeshBody actualPlateBody = new FixedMeshBody ();
      actualPlateBody.setMesh (actualPlateMesh);
      mechModel.addMeshBody (actualPlateBody);

      // Translating FemModel to Actual Box

      RigidTransform3d femToBrickTransform = new RigidTransform3d ();
      PolygonalMesh femMesh = fem.getSurfaceMesh ();

      AffineTransform3d transform = new AffineTransform3d ();
      transform =
         MeshICP.align (brickMesh, femMesh, MeshICP.AlignmentType.RIGID);

      fem.transformGeometry (transform);
      fem.setSurfaceRendering (SurfaceRender.None);
      RenderProps.setLineColor (fem, new Color (1f, 153 / 255f, 153 / 255f));

      // Embedding Mesh
      FemMeshComp meshComp = fem.addMesh (actualPlateMesh);

      for (int i = 0; i < screwMeshes.length; i++) {
         screwMeshes[i] =
            Assist
               .loadGeometry (
                  meshDirectory, "screws" + Integer.toString (i) + ".stl");
         screwMeshes[i].transform (initTransform);
         screwBodies[i] = new FixedMeshBody ("Screw" + i);
         screwBodies[i].setMesh (screwMeshes[i]);
         fem.addMesh (screwMeshes[i]);
         mechModel.addMeshBody (screwBodies[i]);

      }

      // Making nodes reasonable size
      RenderProps newProps = new RenderProps ();
      for (FemNode3d n : fem.getNodes ()) {
         newProps.setPointRadius (0.35);
         n.setRenderProps (newProps);
      }

      int spring_stiffness = 9;
      initializeSpring (0, spring_stiffness);

   }

   public void initializeSpring (int shift_index, int stiff) {
      // Spring Stuff
      // ArrayList<Polyline> lines = PlateMesh.getLines ();
      System.out.println ("init Spring calleddd ");
      Double[] distances = new Double[lines.size ()];
      distances[0] = lines.get (0).computeLength ();
      for (int i = 1; i < lines.size (); i++) {
         distances[i] = distances[i - 1] + lines.get (i).computeLength ();
      }

      // Point3d plateEnd = new Point3d();
      plateStart = lines.get (0).getVertex (0).getWorldPoint ();

      ArrayList<AxialSpring> springList = new ArrayList<AxialSpring> ();
      int idx = 0;
      System.out.println (newrdpMeshBody.numVertices ());

      if (lines
         .get (0).getVertex (0).getWorldPoint ()
         .distance (fem.getNode (0).getPosition ()) > lines
            .get (0).getVertex (0).getWorldPoint ()
            .distance (fem.getNode (plateBendSegments * 4).getPosition ())) {
         // Initial
         Vector3d pullVec = new Vector3d ();
         for (int j = 0; j < 4; j++) {
            springList
               .add (
                  createAxialSpring (
                     fem
                        .getNode (
                           plateBendSegments + j * (plateBendSegments + 1)),
                     lines.get (shift_index).getVertex (0).getWorldPoint (),
                     Assist.GetRigidBody (mechModel, Target)));
            mechModel.addAxialSpring (springList.get (0 * 4 + j));
         }
         for (int i = 1; i < (plateBendSegments + 1); i++) {
            double length = BrickMeshWidths.get (0) / plateBendSegments * i;
            double minDis = 1000;
            double dis = 0;
            for (int j = 0; j < lines.size (); j++) {
               dis = Math.abs (length - distances[j] + distances[shift_index]);
               if (dis < minDis) {
                  minDis = dis;
                  idx = j;
               }
            }

            System.out.println ("index of lines is " + idx);

            for (int j1 = 0; j1 < 4; j1++) {
               springList
                  .add (
                     createAxialSpring (
                        fem
                           .getNode (
                              (plateBendSegments - i)
                              + j1 * (plateBendSegments + 1)),
                        newrdpMeshBody.getVertex (idx).getPosition (),
                        Assist.GetRigidBody (mechModel, Target)));
               mechModel.addAxialSpring (springList.get (i * 4 + j1));
            }
         }
      }
      else {
         // Initial

         Vector3d pullVec = new Vector3d ();
         for (int j = 0; j < 4; j++) {
            springList
               .add (
                  createAxialSpring (
                     fem.getNode (0 + j * (plateBendSegments + 1)),
                     lines.get (shift_index).getVertex (0).getWorldPoint (),
                     Assist.GetRigidBody (mechModel, Target)));
            mechModel.addAxialSpring (springList.get (0 * 4 + j));
         }
         for (int i = 1; i < (plateBendSegments + 1); i++) {
            double length = BrickMeshWidths.get (0) / plateBendSegments * i;
            double minDis = 1000;
            double dis = 0;
            for (int j = 0; j < lines.size (); j++) {
               dis = Math.abs (length - distances[j] + distances[shift_index]);
               if (dis < minDis) {
                  minDis = dis;
                  idx = j;
               }
            }
            if (idx < newrdpMeshBody.getMesh ().getVertices ().size ()
            - length) {
               for (int j1 = 0; j1 < 4; j1++) {
                  springList
                     .add (
                        createAxialSpring (
                           fem.getNode (i + j1 * (plateBendSegments + 1)),
                           newrdpMeshBody.getVertex (idx).getPosition (),
                           Assist.GetRigidBody (mechModel, Target)));
                  mechModel.addAxialSpring (springList.get (i * 4 + j1));
               }
            }
         }
      }
      CollisionManager cm = mechModel.getCollisionManager ();
      cm.setReduceConstraints (true);
      mechModel
         .setCollisionBehavior (
            fem, Assist.GetRigidBody (mechModel, "Recon"), true);

      mechModel.setFriction (0);

      // copied code from updateButtonClicked in plateBendMandible
      double damping = 90;

      RenderableComponentList<AxialSpring> springs = mechModel.axialSprings ();
      System.out.println ("size of spring list is " + springs.size ());
      for (AxialSpring i : springs) {
         i.setDamping (i, damping * 100);
         i.setStiffness (i, stiff * 100);
      }

      System.out
         .println (
            "mechModel has # framemarkers: "
            + mechModel.frameMarkers ().size ());
   }

   public AxialSpring createAxialSpring (
      FemNode3d markerOnFem, Point3d point, RigidBody body) {
      FrameMarker insertion;
      AxialSpring spring = new AxialSpring ();

      insertion = new FrameMarker (point);
      insertion.setFrame (body);
      mechModel.addFrameMarker (insertion);
      plateBendFrameMarkers.add (insertion);

      RenderProps.setLineStyle (spring, LineStyle.CYLINDER);
      // RenderProps.setLineRadius(spring, MEMBRANE_CYL_RADIUS);
      RenderProps.setLineColor (spring, Color.WHITE);
      RenderProps.setLineRadius (spring, 0.3);
      spring.setFirstPoint (markerOnFem);
      spring.setSecondPoint (insertion);
      spring
         .setMaterial (
            new LinearAxialMaterial (/* stiffness= */2500, /* damping= */9000));
      return spring;

   }
}