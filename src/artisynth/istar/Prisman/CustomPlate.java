package artisynth.istar.Prisman;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.lang.model.util.ElementFilter;
import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JCheckBox;

import artisynth.core.femmodels.FemElement;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.workspace.RootModel;
import artisynth.demos.test.IntersectionTester;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.io.GenericMeshWriter;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;

public class CustomPlate extends RootModel {
   MechModel MechMod = new MechModel ();
   ArrayList<RigidBody> holes = new ArrayList<> ();
   ArrayList<RigidBody> plate = new ArrayList<> ();
   ArrayList<RigidBody> Cylinders = new ArrayList<> ();
   RigidBody mandibleBody = new RigidBody ("Mandible");
   PolygonalMesh mandibleMesh = null;
   RigidBody testBody = new RigidBody ("TestBody");
   RigidBody TempRect = new RigidBody ("TempRect");
   IntersectionTester Intersection = null;
   JCheckBox HoleCheckBox;
   Boolean Flag = false;
   int count = 0;
   

   public CustomPlate () {

   }

   private void addControlPanel () {
      ControlPanel myControlPanel = new ControlPanel ("Control Panel", "");
      JButton GoButton = new JButton ("Add Implant");
      JButton DifferenceButton = new JButton ("Create Links");
      JButton ClipButton = new JButton ("Clip");
      HoleCheckBox = new JCheckBox ("Make Screw Hole");
      HoleCheckBox.setSelected (false);
      JButton FinalizeButton = new JButton ("Finalize");
      ClipButton.addActionListener (new ClipButtonClicked ());
      DifferenceButton.addActionListener (new DifferenceButtonClicked ());
      GoButton.addActionListener (new GoButtonClicked ());
      FinalizeButton.addActionListener (new FinalizeButtonClicked ());
      myControlPanel.addWidget (GoButton);
      myControlPanel.addWidget (DifferenceButton);
      myControlPanel.addWidget (ClipButton);
      myControlPanel.addWidget (HoleCheckBox);
      myControlPanel.addWidget (FinalizeButton);
      addControlPanel (myControlPanel);

   }

   public class FinalizeButtonClicked extends AbstractAction {
      PolygonalMesh plateMesh = new PolygonalMesh ();
      SurfaceMeshIntersector intersect = new SurfaceMeshIntersector ();
      RigidBody plateBody = new RigidBody ("PlateBody");

      public FinalizeButtonClicked () {
         putValue (NAME, "Finalize");
      }

      @Override
      public void actionPerformed (ActionEvent e) {
         // TODO Auto-generated method stub
         for (RigidBody obj: plate) {
            try {
               obj.getSurfaceMesh().write (new File ("data/"+obj.getName ()+".stl"));
            }
            catch (IOException e1) {
               System.err.println ("Can't write mesh:");
               e1.printStackTrace();
            }
         }
         for (RigidBody obj : Cylinders) {
            try {
               obj.getSurfaceMesh ().write (new File ("data/"+obj.getName ()+".stl"));
            }
            catch (IOException e1) {
               System.err.println ("Can't write mesh:");
               e1.printStackTrace();
            }
         }
         for (RigidBody obj : plate) {
            System.out.println (obj.getName ());
            // plateBody.addMesh (obj.getSurfaceMesh ());
            // try {
            // GenericMeshWriter.writeMesh (obj.getName ()+".stl",
            // obj.getSurfaceMesh());
            // }catch (IOException E) {
            // E.printStackTrace ();
            // }
            PolygonalMesh mesh = new PolygonalMesh (obj.getSurfaceMesh ());
//            mesh.removeDisconnectedVertices ();
//            mesh.removeDisconnectedFaces ();
            try {
//               plateMesh.removeDisconnectedVertices ();
//               plateMesh.removeDisconnectedFaces ();
               plateMesh.autoGenerateNormals ();
               plateMesh = MeshFactory.getUnion (plateMesh, mesh);
//               plateMesh = intersect.findUnion (mesh, plateMesh);
            }catch(maspack.util.InternalErrorException masspackError) {
               System.out.println(masspackError.getMessage ());
            }
            // plateMesh=MeshFactory.getUnion (plateMesh, obj.getCollisionMesh
            // ());
         }
         // for(PolygonalMesh mesh : plateBody.getSurfaceMeshes ()) {
         //// System.out.println(mesh.getName ());
         //// plateMesh=MeshFactory.getUnion (plateMesh, mesh);
         // plateMesh = intersect.findUnion(mesh, plateMesh);
         // }
         for (RigidBody obj : Cylinders) {
            plateMesh =
               intersect.findDifference01 (plateMesh, obj.getSurfaceMesh ());
            // plateBody.addMesh (obj.getSurfaceMesh ());
            // plateBody.addMesh(obj.getSurfaceMesh ());
            // try {
            // GenericMeshWriter.writeMesh (obj.getName ()+".stl",
            // obj.getSurfaceMesh());
            // }catch (IOException E) {
            // E.printStackTrace ();
            // }

         }
         // RigidBody plateBody = new RigidBody ("PlateBody");
         plateBody.setSurfaceMesh (plateMesh);
         MechMod.clear ();

         MechMod.add (plateBody);
      }

   }

   public class ClipButtonClicked extends AbstractAction {

      public ClipButtonClicked () {
         putValue (NAME, "Clip");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         if (count == 1) {
            convert (0);
         }
         convert (count);
      }

      private void convert (int count) {
         RigidBody ClippedConnector = new RigidBody ("Connector " + count);
         SurfaceMeshIntersector smi =
            new maspack.collision.SurfaceMeshIntersector ();
         try {
         PolygonalMesh connector =
            smi
               .findIntersection (
                  TempRect.getSurfaceMesh (), testBody.getSurfaceMesh ());
         ClippedConnector.setSurfaceMesh (connector);
         }catch(maspack.util.InternalErrorException mError) {
            ClippedConnector.setSurfaceMesh (TempRect.getSurfaceMesh ());
         }

         MechMod.addRigidBody (ClippedConnector);
         plate.add (ClippedConnector);
         holes.get (count).setSurfaceMesh (MeshFactory.createSphere (2.5, 22));
         SurfaceMeshIntersector inter =
            new maspack.collision.SurfaceMeshIntersector ();
         PolygonalMesh circleMesh =
            inter
               .findIntersection (
                  holes.get (count).getSurfaceMesh (),
                  testBody.getSurfaceMesh ());
         RigidBody Connector = new RigidBody ("Sphere " + count);
         Connector.setSurfaceMesh (circleMesh);
         MechMod.addRigidBody (Connector);
         plate.add (Connector);
         if (HoleCheckBox.isSelected ()) {
            PolygonalMesh cylinder =
               new PolygonalMesh (MeshFactory.createCylinder (1, 8, 80));
            RigidBody CylinderBody = new RigidBody ("Cylinder " + count);
            RigidTransform3d transform = new maspack.matrix.RigidTransform3d ();
            transform.R.set (holes.get (count).getRotation ());
            transform.setTranslation (holes.get (count).getPosition ());
            cylinder.transform (transform);
            CylinderBody.setSurfaceMesh (cylinder);
//            CylinderBody.setPose (transform);
            MechMod.add (CylinderBody);
            Cylinders.add (CylinderBody);
         }
         MechMod.removeRigidBody (holes.get (count));
      }
   }

   public class DifferenceButtonClicked extends AbstractAction {

      public DifferenceButtonClicked () {
         putValue (NAME, "Difference");
      }

      private double length (Point3d point1, Point3d point2) {
         double x = (point1.x - point2.x) * (point1.x - point2.x);
         double y = (point1.y - point2.y) * (point1.y - point2.y);
         double z = (point1.z - point2.z) * (point1.z - point2.z);
         return Math.sqrt (x + y + z);
      }

      private Vector3d angle (Point3d point1, Point3d point2) {
         AxialSpring spring = new AxialSpring ("DistanceSpring");
         Point p1 = new Point ();
         Point p2 = new Point ();
         p1.setPosition (point1);
         p2.setPosition (point2);
         spring.setFirstPoint (p1);
         spring.setSecondPoint (p2);
         Vector3d directionVector = spring.getDir ();
         return directionVector;

      }

      @Override
      public void actionPerformed (ActionEvent evt) {
//         removeController (Intersection);
         // System.out.println("Linking: Markers"+count);
         FrameMarker[] frameMarkers = mandibleBody.getFrameMarkers ();
         Point3d point1 = frameMarkers[count].getLocation ();
         Point3d point2 = frameMarkers[count + 1].getLocation ();
         System.out.println ("Now linking");
         System.out.println (point1);
         System.out.println (point2);
         // PolygonalMesh box =
         // MeshFactory.createBox (3, 3, length (point1, point2));
         PolygonalMesh box =
            MeshFactory.createCylinder (2.5, length (point1, point2), 50);
        
         // double[] angles = angle(point1, point2);
         // top,bottom;
         RigidTransform3d transform = new maspack.matrix.RigidTransform3d ();
         transform
            .setTranslation (
               (point1.x + point2.x) / 2, (point1.y + point2.y) / 2,
               (point1.z + point2.z) / 2);
         transform.R.setZDirection (angle (point1, point2));
         // RigidTransform3d transform = new maspack.matrix.RigidTransform3d
         // (0,0,0, angles[1], angles[2], angles[0]);

//         TempRect.setPose (transform);
         box.transform (transform);
         TempRect.setSurfaceMesh (box);
         MechMod.addRigidBody (TempRect);
//         Intersection =
//            new IntersectionTester (
//               testBody.getSurfaceMesh (), TempRect.getSurfaceMesh (), 0);
//         SurfaceMeshIntersector.CSG csg =
//            SurfaceMeshIntersector.CSG.INTERSECTION;
//         Intersection.setCSGOperation (csg);
//         addController (Intersection);
         count = count + 1;
      }
   }

   public class GoButtonClicked extends AbstractAction {
      public GoButtonClicked () {
         putValue (NAME, "Go");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         FrameMarker[] frameMarkers = mandibleBody.getFrameMarkers ();
         int numV = mandibleMesh.numVertices ();
         MultiPointSpring continuousSpring = new MultiPointSpring ();
         continuousSpring.setMaterial (new LinearAxialMaterial (0.1, 5));
         for (int x = 0; x < frameMarkers.length; x++) {
            PolygonalMesh hole = null;
            try {
               hole =
                  new PolygonalMesh (
                     "C:/Users/ascar/git/artisynth_projects/src/artisynth/models/Prisman/untitled.stl");
            }
            catch (IOException e) {
               System.err.println ("Can't read mesh:");
               e.printStackTrace ();
            }
            Point3d point = frameMarkers[x].getLocation ();
            double mindistance = 1000000.0;
            double distance = 0.0;
            int idx1 = 0;
            for (int i = 0; i < numV; i++) {
               Vertex3d v = mandibleMesh.getVertex (i);
               distance = v.distance (point);
               if (distance < mindistance) {
                  mindistance = distance;
                  idx1 = i;
               }
            }
            Vector3d Normal = new Vector3d ();
            Vertex3d vertex1 = mandibleMesh.getVertex (idx1);
            vertex1.computeNormal (Normal);
            RigidBody holeMeshBody = new RigidBody ("hole " + x);
            holes.add (holeMeshBody);
            holeMeshBody.setSurfaceMesh (hole);
            holeMeshBody.setDynamic (Boolean.TRUE);
            RigidTransform3d transform = new maspack.matrix.RigidTransform3d ();
            transform
               .setTranslation (
                  point.x + (Normal.x * 3), point.y + (Normal.y * 3),
                  point.z + (Normal.z * 3));
            transform.R.setZDirection (Normal);
            holeMeshBody.setPose (transform);
            MechMod.addRigidBody (holeMeshBody);
            AxialSpring spring = new AxialSpring ("Spring " + x);
            spring
               .setMaterial (
                  new LinearAxialMaterial (
                     /* stiffness= */5000, /* damping= */1000));
            FrameMarker holemarker =
               new FrameMarker (holeMeshBody, holeMeshBody.getCenterOfMass ());
            MechMod.addFrameMarker (holemarker);
            // spring.setFirstPoint (frameMarkers[x]);
            Point3d pos = holemarker.getPosition ();
            // System.out.println(frameMarkers[x].getLocation ());
            Point pd = new Point ();
            pd.setPosition (pos);

            spring.setFirstPoint (frameMarkers[x]);
            spring.setSecondPoint (holemarker);
            MechMod.addAxialSpring (spring);

            // set render properties for components
            RenderProps.setLineColor (spring, Color.RED);
            RenderProps.setLineWidth (spring, 3);
         }
         // rerender ();
         // MultiPointSpring spring = new MultiPointSpring ();
         // spring.setMaterial (new LinearAxialMaterial (0.1, 5));
         // continuousSpring.addPoint (frameMarkers[0]);
         for (int x = 0; x < frameMarkers.length - 1; x++) {
            continuousSpring.addPoint (frameMarkers[x + 1]);

         }
         MechMod.addMultiPointSpring (continuousSpring);
         MechMod.addRigidBody (testBody);
         rerender ();

      }

   }
   
   public void build (String[] args) {
      RenderProps.setPointStyle (MechMod, PointStyle.SPHERE);
      RenderProps.setPointRadius (MechMod, 1.25);
      MechMod.setGravity (0, 0, 0);
      MechMod.setDefaultCollisionBehavior (true, 0.2);
      // load mandible mesh
      try {
         mandibleMesh =
            new PolygonalMesh (
               // "C:/Users/ascar/git/artisynth_projects/src/artisynth/models/Prisman/MasterMandible.stl");
//               "C:/Users/ascar/Downloads/platereconwoholes.stl");
            "C:/Users/ascar/Downloads/repairmesh.stl");
         // mandibleMesh.removeDisconnectedVertices ();
         // mandibleMesh.removeDisconnectedFaces ();
      }
      catch (IOException e) {
         System.err.println ("Can't read mesh:");
         e.printStackTrace ();
      }
      PolygonalMesh[] mandibleMeshParts =
      mandibleMesh.partitionIntoConnectedMeshes ();
      if (mandibleMeshParts != null) {
         mandibleMesh = new PolygonalMesh(mandibleMeshParts[0]);
         for(PolygonalMesh mesh : mandibleMeshParts) {
            if (mesh.computeArea()> mandibleMesh.computeArea()) {
               mandibleMesh = new PolygonalMesh(mesh);
            }
         }
      }
//      Point3d centroid = new Point3d ();
//      mandibleMesh.computeCentroid (centroid);
//      Vector3d TranslateToCentroidMandible = new Vector3d (centroid).negate ();
//      mandibleMesh.translateToCentroid ();

      mandibleBody.addMesh (mandibleMesh, Boolean.TRUE, Boolean.TRUE);
      MechMod.addRigidBody (mandibleBody);
      mandibleBody.setDynamic (Boolean.FALSE);

      // some fem stuff attempt 1
      // FemModel3d beam = new FemModel3d("beam");
      // double[] size = {8.0, 0.25, 0.25}; // widths
      // int[] res = {10, 2, 2}; // resolution (# elements)
      // FemFactory.createGrid(beam, FemElementType.Hex,
      // size[0], size[1], size[2],
      // res[0], res[1], res[2]);
      // PolygonalMesh rectangle = MeshFactory.createBox (size[0], size[1],
      // size[2]);
      
//      FemModel3d extrusion = new FemModel3d ("extrusion");
//      FemFactory.createExtrusion (extrusion, 1, 2, 2, mandibleMesh);
      // extrusion.
//      MechMod.add (extrusion);
//      PolygonalMesh extrusionMesh = extrusion.createSurfaceMesh (efilter);
      
//       PolygonalMesh extrusionMesh = extrusion.getSurfaceMesh ();
//       extrusionMesh.removeDisconnectedVertices ();
//       extrusionMesh.autoGenerateNormals ();
      
      PolygonalMesh extrusionMesh = null;
      try {
       extrusionMesh = new PolygonalMesh("C:/Users/ascar/Downloads/rip.stl");
      }catch(IOException e) {
         System.out.println(e);
      }
      
//      extrusionMesh.canSelfIntersect = Boolean.FALSE;
//      extrusionMesh.removeDisconnectedVertices ();
//      extrusionMesh.removeDisconnectedFaces ();
      // extrusionmesh = extrusion.set
      // extrusionMesh
      testBody.setSurfaceMesh (extrusionMesh);
      // testBody.setSurfaceMesh (mandibleMesh);
      testBody.setDynamic (Boolean.FALSE);
      testBody.getSurfaceMeshComp ().setIsCollidable (false);
//      MechMod.addRigidBody (testBody);
//      MechMod.remove (extrusion);
      RenderProps.setVisible (testBody, Boolean.FALSE);
      // MechMod.addModel(beam);
      addModel (MechMod);
      addControlPanel ();
   }
}
