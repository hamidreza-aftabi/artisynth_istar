package artisynth.istar.Prisman;

import java.io.*;
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.Iterator;
import java.io.OutputStream;

import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JFileChooser;

import artisynth.core.modelbase.*;
import artisynth.core.renderables.EditablePolygonalMeshComp;
import artisynth.core.mechmodels.*;
import artisynth.core.mechmodels.Collidable.Collidability;
import artisynth.core.workspace.RootModel;
import artisynth.demos.test.MeshTestBase;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.undo.SimpCommand;
import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.*;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.util.*;
import maspack.geometry.*;
import maspack.geometry.MeshICP.AlignmentType;
import maspack.geometry.io.GenericMeshReader;
import maspack.geometry.io.StlReader;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.geometry.io.*;
import maspack.matrix.*;
import maspack.render.*;
import maspack.render.Renderer.LineStyle;
import maspack.collision.*;
import maspack.properties.*;

public class Registration extends RootModel {

   ControlPanel myControlPanel;
   MechModel mechModel;
   
   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();

   JButton registerButton;
   JButton differenceButton;
   JButton transformButton;

   public static String rbpath =
      "G:\\artisynth_projects\\src\\artisynth\\models\\Prisman\\stl\\";

   PolygonalMesh readBoxMesh (String name) {
      PolygonalMesh mesh = null;
      try {
         // mesh = (PolygonalMesh)GenericMeshReader.readMesh(rbpath +
         // "MarzMandible.stl");
         mesh = StlReader.read (rbpath + name);
         System.out.println ("Read Mesh" + name);
      }
      catch (Exception e) {
         System.out.println ("Unable to read mesh: " + name);
         e.printStackTrace ();
         System.exit (1);
      }
      return mesh;
   }

   private void addControlPanel () {

      // Buttons
      myControlPanel = new ControlPanel ("Control Panel", "");

      registerButton = new JButton ("Register");
      registerButton.addActionListener (new registerButtonClicked ());
      
      transformButton = new JButton ("Transform");
      transformButton.addActionListener (new transformClicked ());

      differenceButton = new JButton ("Difference");
      differenceButton.addActionListener (new differenceButtonClicked ());

      myControlPanel.addWidget (transformButton);
      myControlPanel.addWidget (registerButton);
      myControlPanel.addWidget (differenceButton);
      addControlPanel (myControlPanel);

   }

   public class transformClicked extends AbstractAction {
      public transformClicked () {
         putValue (NAME, "Transform");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         Vector3d initialNormal =
            new Vector3d (
               -0.6171081790481495, 0.7735851944621872, -0.1440258388650349);
         Point3d initialPos =
            new Point3d (42.3771941054, 253.89344264, -473.1117075);

         Vector3d newNormal =
            new Vector3d (
               -0.5675099278789908, 0.742446540471231, -0.35595732370760846);
         Point3d newPos = new Point3d (47.06815, 241.71578, -479.26245);

         PolygonalMesh TRScrews = null;
         PolygonalMesh OriScrews = null;

         OriScrews = readBoxMesh ("MandibleGuideLeftScrews.stl");
         FixedMeshBody OriBody = new FixedMeshBody ("OriScrews", OriScrews);
         TRScrews = new PolygonalMesh (OriScrews);
         FixedMeshBody TRBody = new FixedMeshBody ("TRScrews", TRScrews);
         
         Point3d oriCentroid = new Point3d();
         OriScrews.computeCentroid (oriCentroid);
         oriCentroid.transform (OriBody.getPose());
         
         RotationMatrix3d rot = meshHelper.rotatePlane (new Vector3d(initialNormal), new Vector3d(newNormal));
         Vector3d translation = new Vector3d (newPos);
         translation.sub (initialPos);
         RigidTransform3d tr = new RigidTransform3d();
         tr.setRotation (rot);
//         tr.setTranslation (translation);
         
         TRScrews.transform (tr);
         meshHelper.setPlaneOrigin (TRScrews, oriCentroid);
         Point3d newCent = new Point3d (oriCentroid);
         newCent.add (translation);
         meshHelper.setPlaneOrigin (TRScrews, newCent);
         
         mechModel.addMeshBody (TRBody);
         mechModel.addMeshBody (OriBody);
         
         maspack.geometry.io.StlWriter writer = null;
         try {
            // writer =
            // new maspack.geometry.io.StlWriter (rbpath + "lewisoutput.stl");
            // writer.writeMesh (masterInnerGuide);
            writer =
               new maspack.geometry.io.StlWriter (
                  rbpath + "TRLeftScrews.stl");
            writer.writeMesh (TRScrews);
         }
         catch (Exception e) {
            System.out.println ("Unable to save");
         }
         
      }
   }

   public class registerButtonClicked extends AbstractAction {
      public registerButtonClicked () {
         putValue (NAME, "Register");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         // PolygonalMesh mesh1 = null;
         // PolygonalMesh mesh2 = null;
         PolygonalMesh mandible = null;
         PolygonalMesh masterMandible = null;
         PolygonalMesh masterInnerGuide = null;

         mandible = readBoxMesh ("NunnMirroredLowPoly.stl");
         masterMandible = readBoxMesh ("MarzMandibleAsMaster.stl");
         masterInnerGuide = readBoxMesh ("MasterInnerGuide.stl");

         FixedMeshBody masterMandibleBody =
            new FixedMeshBody ("MasterMandible", masterMandible);
         FixedMeshBody mandibleBody = new FixedMeshBody ("mandible", mandible);
         FixedMeshBody masterInnerGuideBody =
            new FixedMeshBody ("MasterGuide", masterInnerGuide);

         Vector3d v1master = new Vector3d ();
         Point3d centroidMaster = new Point3d ();
         masterMandible.computeCentroid (centroidMaster);
         // RigidTransform3d poseMaster = MasterMandible.getPose ();
         // centroidMaster.transform(poseMaster);
         Point3d massMaster = new Point3d ();
         masterMandible.computeCentreOfVolume (massMaster);
         // massMaster.transform (poseMaster);
         v1master = new Vector3d (massMaster);
         v1master.sub (centroidMaster);
         v1master.normalize ();

         Vector3d v1patient = new Vector3d ();
         Point3d centroidPatient = new Point3d ();
         mandible.computeCentroid (centroidPatient);
         // RigidTransform3d poseMaster = MasterMandible.getPose ();
         // centroidMaster.transform(poseMaster);
         Point3d massPatient = new Point3d ();
         mandible.computeCentreOfVolume (massPatient);
         // massMaster.transform (poseMaster);
         v1patient = new Vector3d (massPatient);
         v1patient.sub (centroidPatient);
         v1patient.normalize ();

         // PolygonalMesh centervol = MeshFactory.createBox (5, 5, 5,
         // massPatient.get (0), massPatient.get (1),
         // massPatient.get (2));
         // FixedMeshBody ctrvB = new FixedMeshBody ("Patient Volume",
         // centervol);
         // mechModel.addMeshBody (ctrvB);
         //
         // PolygonalMesh centroid = MeshFactory.createBox (5, 5, 5,
         // centroidPatient.get (0), centroidPatient.get (1),
         // centroidPatient.get (2));
         // FixedMeshBody centroidB = new FixedMeshBody ("Patient centroid",
         // centroid);
         // mechModel.addMeshBody (centroidB);

         // System.out.println (v1master);
         // System.out.println (v1patient);

         Vector3d v2master = new Vector3d ();
         int num = masterMandible.numVertices ();
         ArrayList<Point3d> masterPoints = new ArrayList<Point3d> ();
         double max = 0;
         int id = 0;
         for (int i = 0; i < num; i++) {
            masterPoints.add (i, masterMandible.getVertex (i).getPosition ());
            Vector3d point = masterMandible.getVertex (i).getPosition ();
            double distance = centroidMaster.distance (point);
            if (distance > max) {
               max = distance;
               id = i;
            }
         }
         v2master = new Vector3d (masterMandible.getVertex (id).getPosition ());
         Vector3d mcondyle1 =
            new Vector3d (masterMandible.getVertex (id).getPosition ());

         int id2 = 0;
         max = 0;
         for (int i = 0; i < num; i++) {
            Vector3d point = masterMandible.getVertex (i).getPosition ();
            double distance = centroidMaster.distance (point);
            double pp = point.distance (mcondyle1);
            if ((distance > max) && (pp > 25)) {
               max = distance;
               id2 = i;
            }
         }
         v2master =
            new Vector3d (masterMandible.getVertex (id2).getPosition ());
         Vector3d mcondyle2 =
            new Vector3d (masterMandible.getVertex (id2).getPosition ());
         v2master.sub (mcondyle1);
         v2master.normalize ();

         System.out.println (id);
         System.out.println (id2);

         Vector3d v2patient = new Vector3d ();
         num = mandible.numVertices ();
         ArrayList<Point3d> patientPoints = new ArrayList<Point3d> ();
         max = 0;
         id = 0;
         for (int i = 0; i < num; i++) {
            patientPoints.add (i, mandible.getVertex (i).getPosition ());
            Vector3d point = mandible.getVertex (i).getPosition ();
            double distance = centroidPatient.distance (point);
            if (distance > max) {
               max = distance;
               id = i;
            }
         }
         Vector3d condyle1 =
            new Vector3d (mandible.getVertex (id).getPosition ());
         v2patient = new Vector3d (mandible.getVertex (id).getPosition ());
         // v2patient.sub (centroidPatient);
         // v2patient.normalize();

         num = mandible.numVertices ();
         max = 0;
         id2 = 0;
         for (int i = 0; i < num; i++) {
            Vector3d point = mandible.getVertex (i).getPosition ();
            double distance = centroidPatient.distance (point);
            double pp = point.distance (condyle1);
            if ((distance > max) && (pp > 25)) {
               max = distance;
               id2 = i;
            }
         }
         Vector3d condyle2 =
            new Vector3d (mandible.getVertex (id2).getPosition ());
         v2patient = new Vector3d (mandible.getVertex (id2).getPosition ());
         v2patient.sub (condyle2);
         v2patient.normalize ();
         System.out.println (id);
         System.out.println (id2);

         // PolygonalMesh c1 =
         // MeshFactory.createBox (
         // 5, 5, 5, condyle1.get (0), condyle1.get (1), condyle1.get (2));
         // FixedMeshBody c1B = new FixedMeshBody ("PatientC1", c1);
         // mechModel.addMeshBody (c1B);
         //
         // PolygonalMesh c2 =
         // MeshFactory.createBox (
         // 5, 5, 5, condyle2.get (0), condyle2.get (1), condyle2.get (2));
         // FixedMeshBody c2B = new FixedMeshBody ("PatientC2", c2);
         // mechModel.addMeshBody (c2B);
         //
         // PolygonalMesh centroid =
         // MeshFactory.createBox (
         // 5, 5, 5, centroidPatient.get (0), centroidPatient.get (1),
         // centroidPatient.get (2));
         // FixedMeshBody centroidB =
         // new FixedMeshBody ("Patient centroid", centroid);
         // mechModel.addMeshBody (centroidB);
         //
         // PolygonalMesh mc1 =
         // MeshFactory.createBox (
         // 5, 5, 5, mcondyle1.get (0), mcondyle1.get (1),
         // mcondyle1.get (2));
         // FixedMeshBody mc1B = new FixedMeshBody ("MasterC1", mc1);
         // mechModel.addMeshBody (mc1B);
         //
         // PolygonalMesh mc2 =
         // MeshFactory.createBox (
         // 5, 5, 5, mcondyle2.get (0), mcondyle2.get (1),
         // mcondyle2.get (2));
         // FixedMeshBody mc2B = new FixedMeshBody ("MasterC2", mc2);
         // mechModel.addMeshBody (mc2B);
         //
         // PolygonalMesh centroidM =
         // MeshFactory.createBox (
         // 5, 5, 5, centroidMaster.get (0), centroidMaster.get (1),
         // centroidMaster.get (2));
         // FixedMeshBody centroidMB =
         // new FixedMeshBody ("Master centroid", centroidM);
         // mechModel.addMeshBody (centroidMB);

         Vector3d patientVector = new Vector3d (condyle1);
         patientVector.sub (centroidPatient);
         patientVector.normalize ();

         Vector3d masterVector = new Vector3d (mcondyle2);
         masterVector.sub (centroidMaster);
         masterVector.normalize ();

         Point3d[] ptfPatient = new Point3d[3];
         ptfPatient[0] = new Point3d (condyle1);
         ptfPatient[1] = new Point3d (condyle2);
         ptfPatient[2] = new Point3d (centroidPatient);

         Point3d[] ptfMaster = new Point3d[3];
         ptfMaster[0] = new Point3d (mcondyle2);
         ptfMaster[1] = new Point3d (mcondyle1);
         ptfMaster[2] = new Point3d (centroidMaster);

         RigidTransform3d align = SVDRegistration (ptfPatient, ptfMaster);
         masterMandible.transform (align);
         masterInnerGuide.transform (align);

         v1master = new Vector3d ();
         centroidMaster = new Point3d ();
         masterMandible.computeCentroid (centroidMaster);
         // RigidTransform3d poseMaster = MasterMandible.getPose ();
         // centroidMaster.transform(poseMaster);
         massMaster = new Point3d ();
         masterMandible.computeCentreOfVolume (massMaster);
         // massMaster.transform (poseMaster);
         v1master = new Vector3d (massMaster);
         v1master.sub (centroidMaster);
         v1master.normalize ();

         v1patient = new Vector3d ();
         centroidPatient = new Point3d ();
         mandible.computeCentroid (centroidPatient);
         // RigidTransform3d poseMaster = MasterMandible.getPose ();
         // centroidMaster.transform(poseMaster);
         massPatient = new Point3d ();
         mandible.computeCentreOfVolume (massPatient);
         // massMaster.transform (poseMaster);
         v1patient = new Vector3d (massPatient);
         v1patient.sub (centroidPatient);
         v1patient.normalize ();

         double compare = v1patient.dot (v1master);
         compare = compare / (v1patient.norm () * v1master.norm ());

         if (compare > Math.cos (Math.toRadians (15))) {
            System.out.println ("Need to flip masters upside down");
         }

         PolygonalMesh c2 =
            MeshFactory.createBox (
               5, 5, 5, massPatient.get (0), massPatient.get (1),
               massPatient.get (2));
         FixedMeshBody c2B = new FixedMeshBody ("volumePatient", c2);
         // mechModel.addMeshBody (c2B);

         PolygonalMesh centroid =
            MeshFactory.createBox (
               5, 5, 5, centroidPatient.get (0), centroidPatient.get (1),
               centroidPatient.get (2));
         FixedMeshBody centroidB =
            new FixedMeshBody ("Patient centroid", centroid);
         // mechModel.addMeshBody (centroidB);

         PolygonalMesh mc1 =
            MeshFactory.createBox (
               5, 5, 5, massMaster.get (0), massMaster.get (1),
               massMaster.get (2));
         FixedMeshBody mc1B = new FixedMeshBody ("volumeMaster", mc1);
         // mechModel.addMeshBody (mc1B);

         PolygonalMesh centroidM =
            MeshFactory.createBox (
               5, 5, 5, centroidMaster.get (0), centroidMaster.get (1),
               centroidMaster.get (2));
         FixedMeshBody centroidMB =
            new FixedMeshBody ("Master centroid", centroidM);
         // mechModel.addMeshBody (centroidMB);

         // long startTime = System.nanoTime ();
         // maspack.geometry.CPD cpd = new maspack.geometry.CPD ();
         // maspack.matrix.AffineTransform3d transform = null;
         // transform = CPD.affine (mandible, masterMandible, 0, 0.01, 500);
         // masterMandible.transform (transform);
         // masterInnerGuide.transform (transform);
         // long endTime = System.nanoTime ();
         // long duration = (endTime - startTime) / 1000000; // divide by
         // 1000000 to get milliseconds.
         //
         // masterMandible.flip ();
         // masterInnerGuide.flip ();
         //
         // cpd = new maspack.geometry.CPD ();
         // transform = CPD.affine (mandible, masterMandible, 0, 0.01, 500);
         // masterMandible.transform (transform);
         // masterInnerGuide.transform (transform);
         //
         // System.out.println ("Duration:" + duration);
         // masterMandible.transform (transform);
         // masterInnerGuide.transform (transform);
         // masterInnerGuide.autoGenerateNormals ();

         maspack.geometry.io.StlWriter writer = null;
         try {
            // writer =
            // new maspack.geometry.io.StlWriter (rbpath + "lewisoutput.stl");
            // writer.writeMesh (masterInnerGuide);
            writer =
               new maspack.geometry.io.StlWriter (
                  rbpath + "NunnRegisteredRegistered.stl");
            writer.writeMesh (masterMandible);
         }
         catch (Exception e) {
            System.out.println ("Unaable to save");
         }

         // mechModel.setGravity(0, 0, 0);
         // FemModel3d fem = new FemModel3d ("FEM");
         // mechModel.add (fem);
         // FemMeshComp masters;
         //
         // PolygonalMesh combinedMasters =
         // new PolygonalMesh (masterMandible.clone ());
         // combinedMasters.addMesh (masterInnerGuide.clone ());
         // OBB combinedMastersOBB = combinedMasters.computeOBB ();
         // Vector3d widthsMasters = new Vector3d ();
         // combinedMastersOBB.getWidths (widthsMasters);
         // System.out.println (widthsMasters);
         // FixedMeshBody combinedBody = new FixedMeshBody ("CombinedBody",
         // combinedMasters);
         //
         // //mechModel.addMeshBody (combinedBody);
         //
         //
         // // Build hex beam and set properties
         //// double[] size = { widthsMasters.get (0)+20, widthsMasters.get
         // (1)+20, 20+widthsMasters.get (2) };
         // double[] size = {260, 32, 73};
         // int[] res = { 30, 30, 30 };
         //// FemFactory.createFromMesh (fem, combinedMasters, 0);
         //// FemFactory.createExtrusion (fem, 2, 0.0005, 0.0005,
         // combinedMasters);
         // FemFactory.createHexGrid (
         // fem, size[0], size[1], size[2], res[0], res[1], res[2]);
         // fem.setParticleDamping (2);
         // fem.setDensity (10);
         // fem.setMaterial (new LinearMaterial (15000, 0.33));
         // fem.transformGeometry (combinedMastersOBB.getTransform ());
         //
         // // Add embedded meshes
         // masters = fem.addMesh (masterInnerGuide);
         // masters = fem.addMesh (masterMandible);
         // masters.setCollidable (Collidability.EXTERNAL);
         //
         // // Boundary condition: fixed LHS
         // for (FemNode3d node : fem.getNodes ()) {
         //// node.setPosition (p);
         //// node.setRestPosition (pos);
         //
         // }
         //
         // FemMeshICPController controller = new FemMeshICPController(masters,
         // mandible);
         // controller.setName("registration");
         // controller.setPressureFunction(new LogNormalPressureFunction(15000,
         // 2, 0.01, false));
         // addController(controller);
         //
         // // Set rendering properties
         // RenderProps.setAlpha(mandibleBody, 0.9);
         // RenderProps.setFaceColor(mandibleBody, Color.CYAN);
         // RenderProps.setAlpha(masters, 1);
         // RenderProps.setFaceColor(masters, Color.RED);
         //
         // RenderProps.setVisible(controller, false);
         // controller.setForceRenderScale(0.001);
         // RenderProps.setLineStyle(controller, LineStyle.SOLID_ARROW);
         // RenderProps.setLineColor(controller, Color.BLACK);
         // RenderProps.setLineRadius(controller, 0.01);

         // FixedMeshBody meshPlaneBody = new FixedMeshBody (meshPlane);
         mechModel.addMeshBody (masterMandibleBody);
         mechModel.addMeshBody (mandibleBody);
         mechModel.addMeshBody (masterInnerGuideBody);

         // mechModel.addMeshBody (meshPlaneBody);
         getMainViewer ().autoFit ();
      }
   }

   // FEM render properties
   protected void setFemRenderProps (FemModel3d fem) {
      fem.setSurfaceRendering (SurfaceRender.Shaded);
      RenderProps.setLineColor (fem, Color.GREEN);
      RenderProps.setFaceColor (fem, new Color (0.5f, 0.5f, 1f));
      RenderProps.setAlpha (fem, 0.2); // transparent
   }

   // FemMeshComp render properties
   protected void setMeshRenderProps (FemMeshComp mesh) {
      mesh.setSurfaceRendering (SurfaceRender.Shaded);
      RenderProps.setFaceColor (mesh, new Color (1f, 0.5f, 0.5f));
      RenderProps.setAlpha (mesh, 1.0); // opaque
   }

   public class differenceButtonClicked extends AbstractAction {
      public differenceButtonClicked () {
         putValue (NAME, "Difference");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         PolygonalMesh guide = readBoxMesh ("Guide.stl");
         PolygonalMesh cube = readBoxMesh ("Cube.stl");
         PolygonalMesh differenced = new PolygonalMesh (guide.clone ());
         differenced = MeshFactory.getSubtraction (differenced, cube.clone ());
         maspack.geometry.io.StlWriter writer = null;
         try {
            writer =
               new maspack.geometry.io.StlWriter (rbpath + "JoneNewGuide.stl");
            writer.writeMesh (differenced);
         }
         catch (Exception e) {
            System.out.println ("Unaable to save");
         }
      }
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
      SVDecomposition SVD = new SVDecomposition (matH);
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

   @Override
   public void build (String[] args) {
      mechModel = new MechModel ("mechModel");
      addModel (mechModel);
      addControlPanel ();
   }
}
