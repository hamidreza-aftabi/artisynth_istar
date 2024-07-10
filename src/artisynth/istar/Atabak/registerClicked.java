package artisynth.istar.Atabak;

import java.awt.event.ActionEvent;
import java.io.File;
import java.util.ArrayList;

import javax.swing.AbstractAction;
import javax.swing.JCheckBox;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.util.ArtisynthPath;
import maspack.geometry.CPD;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;

public class registerClicked extends AbstractAction {
   ReconstructionModel root = null;
   MechModel mechModel = null;
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   Assist Assist = new Assist ();
   JCheckBox platingDir;

   public registerClicked (JCheckBox platingDir, ReconstructionModel root, MechModel mechModel) {
      putValue (NAME, "Registration");
      this.root = root;
      this.mechModel = mechModel;
      this.platingDir = platingDir;
   }

   @Override
   public void actionPerformed (ActionEvent evt) {
      String homedir = ArtisynthPath.getHomeDir ();
      File pathHome = new File (homedir);
      String homeParent = pathHome.getParentFile ().getAbsolutePath ();
      if (root.DEBUG) {
         System.out.println ("Parent Directory: " + homeParent);
      }

      String masterMandiblePath =
         homeParent
         + "/artisynth_projects/src/artisynth/models/Prisman/stl/MasterMandible.stl";
      PolygonalMesh masterMandibleMesh =
         meshHelper.readMesh (masterMandiblePath, "MasterMandible.stl");
      PolygonalMesh mandibleMesh = Assist.GetMesh (mechModel, "Mandible");
      System.out.println ("Registration Begin");
      FixedMeshBody MasterMandible =
         new FixedMeshBody ("Master Mandible", masterMandibleMesh);
      mechModel.addMeshBody (MasterMandible);

      String guidePath =
         homeParent
         + "/artisynth_projects/src/artisynth/models/Prisman/stl/MasterGuidesThin.stl";
      PolygonalMesh masterGuideMesh =
         meshHelper.readMesh (guidePath, "MasterGuidesThin.stl");
      // MasterGuide = new FixedMeshBody ("Master Guide", masterGuideMesh);
      FixedMeshBody MasterGuide =
         new FixedMeshBody ("Master Guide", masterGuideMesh);
      mechModel.addMeshBody (MasterGuide);

      // Registering Master Mandible and Guide
      // First, transforming master mandible and guide so that it faces the
      // same way as patient's mandible
      Point3d centroidMaster = new Point3d ();
      masterMandibleMesh.computeCentroid (centroidMaster);
      Point3d centroidPatient = new Point3d ();
      Assist.GetMesh (mechModel, "Mandible").computeCentroid (centroidPatient);

      int n = masterMandibleMesh.numVertices ();
      ArrayList<Point3d> masterPoints = new ArrayList<Point3d> ();
      double max = 0;
      int id = 0;
      double distance = 0;
      Vector3d point = null;
      for (int i = 0; i < n; i++) {
         masterPoints.add (i, masterMandibleMesh.getVertex (i).getPosition ());
         point = masterMandibleMesh.getVertex (i).getPosition ();
         distance = centroidMaster.distance (point);
         if (distance > max) {
            max = distance;
            id = i;
         }
      }
      point = null;
      distance = 0;

      Vector3d mcondyle1 =
         new Vector3d (masterMandibleMesh.getVertex (id).getPosition ());

      int id2 = 0;
      max = 0;
      for (int i = 0; i < n; i++) {
         point = masterMandibleMesh.getVertex (i).getPosition ();
         distance = centroidMaster.distance (point);
         double pp = point.distance (mcondyle1);
         if ((distance > max) && (pp > 25)) {
            max = distance;
            id2 = i;
         }
      }

      point = null;
      distance = 0;

      Vector3d mcondyle2 =
         new Vector3d (masterMandibleMesh.getVertex (id2).getPosition ());

      String condyle = "Left";
      if (id2 < id) {
         condyle = "right";
      }

      Vector3d v2patient = new Vector3d ();
      n = Assist.GetMesh (mechModel, "Mandible").numVertices ();
      ArrayList<Point3d> patientPoints = new ArrayList<Point3d> ();
      max = 0;
      id = 0;
      for (int i = 0; i < n; i++) {
         patientPoints.add (i, mandibleMesh.getVertex (i).getPosition ());
         point = mandibleMesh.getVertex (i).getPosition ();
         distance = centroidPatient.distance (point);
         if (distance > max) {
            max = distance;
            id = i;
         }
      }

      point = null;
      distance = 0;

      Vector3d condyle1 =
         new Vector3d (mandibleMesh.getVertex (id).getPosition ());

      n = mandibleMesh.numVertices ();
      max = 0;
      id2 = 0;
      for (int i = 0; i < n; i++) {
         point = mandibleMesh.getVertex (i).getPosition ();
         distance = centroidPatient.distance (point);
         double pp = point.distance (condyle1);
         if ((distance > max) && (pp > 25)) {
            max = distance;
            id2 = i;
         }
      }

      point = null;
      distance = 0;

      Vector3d condyle2 =
         new Vector3d (mandibleMesh.getVertex (id2).getPosition ());
      PolylineMesh PlateMesh = (PolylineMesh)Assist.GetNonPolyMesh (mechModel, "Plate");
      String platedir = "rightleft";
      if (platingDir.isSelected ()) {
         platedir = "leftright";
      }
      if (platedir == "leftright") {
         double dLeft =
            PlateMesh.getVertex (0).getPosition ().distance (condyle1);
         int last = PlateMesh.numVertices ();
         double dRight =
            PlateMesh.getVertex (last - 1).getPosition ().distance (condyle1);
         if ((dLeft > dRight) && (condyle == "left")) {
            Point3d tempCon = new Point3d (condyle1);
            condyle1 = new Point3d (condyle2);
            condyle2 = new Point3d (tempCon);
         }
         else if ((dRight > dLeft) && (condyle == "right")) {
            Point3d tempCon = new Point3d (condyle1);
            condyle1 = new Point3d (condyle2);
            condyle2 = new Point3d (tempCon);
         }
      }

      else {
         double dRight =
            PlateMesh.getVertex (0).getPosition ().distance (condyle1);
         int last = PlateMesh.numVertices ();
         double dLeft =
            PlateMesh.getVertex (last - 1).getPosition ().distance (condyle1);
         if ((dLeft > dRight) && (condyle == "left")) {
            Point3d tempCon = new Point3d (condyle1);
            condyle1 = new Point3d (condyle2);
            condyle2 = new Point3d (tempCon);
         }
         else if ((dRight > dLeft) && (condyle == "right")) {
            Point3d tempCon = new Point3d (condyle1);
            condyle1 = new Point3d (condyle2);
            condyle2 = new Point3d (tempCon);
         }
      }

      Point3d[] ptfPatient = new Point3d[3];
      ptfPatient[0] = new Point3d (condyle1);
      ptfPatient[1] = new Point3d (condyle2);
      ptfPatient[2] = new Point3d (centroidPatient);

      Point3d[] ptfMaster = new Point3d[3];
      ptfMaster[0] = new Point3d (mcondyle1);
      ptfMaster[1] = new Point3d (mcondyle2);
      ptfMaster[2] = new Point3d (centroidMaster);

      RigidTransform3d align =
         meshHelper.SVDRegistration (ptfPatient, ptfMaster);
      masterMandibleMesh.transform (align);
      masterGuideMesh.transform (align);

      maspack.matrix.AffineTransform3d transform = null;
      transform = CPD.affine (mandibleMesh, masterMandibleMesh, 0, 0.01, 500);
      masterMandibleMesh.transform (transform);
      masterGuideMesh.transform (transform);
      masterGuideMesh.autoGenerateNormals ();

      MasterMandible.getRenderProps ().setVisible (false);

      System.out.println ("Registration End");
   }
}