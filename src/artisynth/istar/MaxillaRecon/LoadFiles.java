package artisynth.istar.MaxillaRecon;

import java.awt.event.ActionEvent;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.*;
import javax.swing.AbstractAction;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.util.*;
import artisynth.core.driver.*;
import artisynth.istar.Assist.Assist;
import artisynth.istar.Prisman.HelperMeshFunctions;
import artisynth.istar.Prisman.ReconstructionModel;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AxisAngle;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;
import maspack.util.*;
import maspack.widgets.*;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;

// Load function loads file chosen by user into the program
public class LoadFiles extends AbstractAction {
   MechModel model = null;
   JComboBox loads = null;
   ReconstructionModel root = null;
   JFileChooser fileChooser = new JFileChooser ();
   Assist Assist = new Assist ();
   Boolean slicerPlanes;
   Boolean blenderPlanes;
   File setPlaneFile = null;
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();

   // Change MandibleReconstruction to Root Model once the appropriate constants
   // r extracted
   public LoadFiles (ReconstructionModel root, MechModel model,
   JComboBox loads) {
      putValue (NAME, "Load Stl Files");
      this.model = model;
      this.loads = loads;
      this.root = root;
   }

   @Override
   public void actionPerformed (ActionEvent evt) {

      // Loads Mandible, Donor, Fiducials, and plane values all at once
      // for Artisynth
      switch ((String)loads.getSelectedItem ()) {
         case ("Maxilla"): {
            System.out.println (loads.getSelectedItem ().toString ());
            int returnVal = fileChooser.showOpenDialog (fileChooser);

            if (returnVal == JFileChooser.APPROVE_OPTION) {
               File file = fileChooser.getSelectedFile ();
               // Load mandible mesh from file and center to Artisynth origin
               RigidBody maxillaBody =
                  Assist.FileToRigidBody (file, "Maxilla", true);
               model.addRigidBody (maxillaBody);
               // Add mandible to visibility panel
               Assist
                  .attachVisiblityToPanel (
                     maxillaBody, root.getControlPanels ().get ("Visibility"));
               // Set the origin of the entire model
               root.TranslateToCentroidMandible = Assist.CenterTransform (file);
               // mandibleMesh.translateToCentroid ();
               root.getMainViewer ().autoFit ();
               root.rerender ();

            }
            else {
               System.out.println ("Open command cancelled by user.");
            }
            break;
         }
         case ("Planes"): {
            System.out.println (loads.getSelectedItem ().toString ());
            int returnVal1 = fileChooser.showOpenDialog (fileChooser);
            if (returnVal1 == JFileChooser.APPROVE_OPTION) {
               setPlaneFile = fileChooser.getSelectedFile ();
               createPlanes (root, model);
            }
            break;
         }
         case("Donor"): {
            System.out.println (loads.getSelectedItem ().toString ());
            int returnVal2 = fileChooser.showOpenDialog (fileChooser);
            if (returnVal2 == JFileChooser.APPROVE_OPTION) {
               File DonorFile = fileChooser.getSelectedFile ();
               loadDonor(DonorFile);
            }
            break;
         }
         case("Markers"): {
            Main main = Main.getMain();
            if (model.rigidBodies().get("Maxilla") == null) {
               GuiUtils.showError (main.getMainFrame(), "Maxilla not present");
               return;
            }
            System.out.println (loads.getSelectedItem ().toString ());
            int retval = fileChooser.showDialog (main.getMainFrame(), "Load");
            if (retval == JFileChooser.APPROVE_OPTION) {
               File file = fileChooser.getSelectedFile ();
               loadMaxillaMarkers (file);
            }
            break;
         }
      }
   }

   private void loadFramemarkers (File platefile, Boolean isSlicer) {
      root.frameMarkerPos.clear ();
      root.fm.clear ();
      try {
         FileInputStream input = new FileInputStream (platefile);
         DataInputStream in = new DataInputStream (input);
         BufferedReader br = new BufferedReader (new InputStreamReader (in));
         String strLine;
         int j = 0;
         while ((strLine = br.readLine ()) != null) {
            String[] tokens = strLine.split (" ");
            Double el1 = Double.parseDouble (tokens[0]);
            Double el2 = Double.parseDouble (tokens[1]);
            Double el3 = Double.parseDouble (tokens[2]);
            Point3d pos = new Point3d (el1, el2, el3);
            if (isSlicer) {
               pos.add (root.TranslateToCentroidMandible);
            }
            root.frameMarkerPos.add (j, pos);
            j++;
         }
         int size = root.frameMarkerPos.size ();
         for (int k = 0; k < size; k++) {
            root.fm.add (k, new FrameMarker (root.frameMarkerPos.get (k)));
            root.fm.get (k).setFrame (Assist.GetRigidBody (model, "Mandible"));
            RenderProps rp = new RenderProps ();
            RenderProps.setPointStyle (model, PointStyle.SPHERE);
            RenderProps.setPointRadius (model, 1.25);
            root.fm.get (k).setRenderProps (rp);
            model.addFrameMarker (root.fm.get (k));
         }

         root.rerender ();

      }
      catch (IOException e) {
         System.out.print ("Exception");
      }

   }

   private void loadDonor (File file) {
      // LoadingMesh
      // DonorMesh =
      // meshHelper.readMesh (file.getAbsolutePath (), file.getName ());
      FixedMeshBody DonorMeshBody =
         Assist.FileToFixedMeshBody (file, "Donor", true);
      model.addMeshBody (DonorMeshBody);
      root.rerender ();

      // Editing Prep Donor Values
//      if (root.donorIsScapulaCheckBox.isSelected ()) {
//         // what does this do??? FIgure it out later
//         // root.DonorDistanceProx.setValue (50);
//         // root.DonorDistanceDis.setValue (0.01);
//         // root.multiplier.setValue (0.5);
//         //
//         Vector3d mandibleCentroid = new Vector3d ();
//         // TODO: Fix this line
//         model
//            .rigidBodies ().get ("Mandible").getSurfaceMesh ()
//            .computeCentroid (mandibleCentroid);
//         RigidTransform3d transform = new maspack.matrix.RigidTransform3d ();
//         transform.setTranslation (mandibleCentroid);
//         root.scapulaTrimPlane = MeshFactory.createPlane (250, 90, 10, 10);
//         FixedMeshBody scapulaTrimPlaneBody =
//            new FixedMeshBody ("ScapulaTrimPlane", root.scapulaTrimPlane);
//         model.addMeshBody (scapulaTrimPlaneBody);
//         scapulaTrimPlaneBody.transformGeometry (transform);
//      }
   }

   private void loadMandible (File file) {
      // Load mandible mesh from file and center to Artisynth origin
      RigidBody mandibleBody = Assist.FileToRigidBody (file, "Mandible", true);
      model.addRigidBody (mandibleBody);
      // Add mandible to visibility panel
      Assist
         .attachVisiblityToPanel (
            mandibleBody, root.getControlPanels ().get ("Visibility"));
      // Set the origin of the entire model
      root.TranslateToCentroidMandible = Assist.CenterTransform (file);
      // mandibleMesh.translateToCentroid ();
      root.getMainViewer ().autoFit ();
      root.rerender ();
   }

   private void createPlanes (ReconstructionModel root, MechModel mechModel) {
      Vector3d maxillaCentroid = new Vector3d ();
      Assist.GetMesh (mechModel, "Maxilla").computeCentroid (maxillaCentroid);
      try {
         FileInputStream inputPlane = new FileInputStream (setPlaneFile);
         DataInputStream in = new DataInputStream (inputPlane);
         BufferedReader br = new BufferedReader (new InputStreamReader (in));
         String strLine;
         int i = 0;
         while ((strLine = br.readLine ()) != null) {
            String[] normal = strLine.split (" ");
            strLine = br.readLine ();
            String[] center = strLine.split (" ");
            Vector3d normalPlane =
               new Vector3d (
                  Double.parseDouble (normal[0]),
                  Double.parseDouble (normal[1]),
                  Double.parseDouble (normal[2]));
            Point3d centerPlane =
               new Point3d (
                  Double.parseDouble (center[0]),
                  Double.parseDouble (center[1]),
                  Double.parseDouble (center[2]));
            PolygonalMesh planeMesh = MeshFactory.createPlane (200, 200, 10, 10);
            FixedMeshBody planeMeshBody =
               new FixedMeshBody ("plane" + i, planeMesh);
            planeMesh.translateToCentroid ();
            RigidTransform3d transform = new RigidTransform3d ();
            transform.setTranslation (maxillaCentroid);
            planeMesh.transform (transform);
            Vector3d currentPlaneNormal = planeMesh.getNormal (0);
            RigidTransform3d pose = planeMeshBody.getPose ();
            currentPlaneNormal.transform (pose);
            RotationMatrix3d rotatePlane =
               meshHelper.rotatePlane (currentPlaneNormal, normalPlane);
            AffineTransform3d t = new AffineTransform3d ();
            t.setRotation (rotatePlane);
            planeMeshBody.transformGeometry (t);
            Assist.setPlaneOrigin (planeMeshBody, centerPlane);
            mechModel.addMeshBody (planeMeshBody);
            root.activePlaneList.add (planeMeshBody);
            i++;
         }
      }
      catch (IOException e) {
         System.out.println ("Exception");
      }
   }
   
   void loadMaxillaMarkers (File file) {
      RigidBody maxilla = model.rigidBodies().get("Maxilla");
      if (maxilla == null) {
         System.out.println ("Error, maxilla is not present");
         return;
      }
      ReaderTokenizer rtok = null;
      ArrayList<Point3d> points = new ArrayList<>();
      try {
         rtok = ArtisynthIO.newReaderTokenizer (file);
         while (rtok.nextToken() != ReaderTokenizer.TT_EOF) {
            rtok.pushBack();
            Point3d pnt = new Point3d();
            pnt.x = rtok.scanNumber();
            pnt.y = rtok.scanNumber();
            pnt.z = rtok.scanNumber();
            points.add (pnt);
         }
      }
      catch (IOException e) {
         System.out.println ("Error reading "+file+": " + e);
         return;
      }
      finally {
         if (rtok != null) {
            rtok.close();
         }
      }     
      model.clearFrameMarkers();
      for (Point3d pnt : points) {
         model.addFrameMarkerWorld (maxilla, pnt);
      }
   }
}
