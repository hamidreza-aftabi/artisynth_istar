package artisynth.istar.MandibleRecon;

import java.awt.event.ActionEvent;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;

import javax.swing.AbstractAction;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
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
      if (loads.getSelectedIndex () == 1) {

         fileChooser.setMultiSelectionEnabled (true);
         int returnVal = fileChooser.showOpenDialog (fileChooser);

         if (returnVal == JFileChooser.APPROVE_OPTION) {
            File[] files = fileChooser.getSelectedFiles ();

            for (int i = 0; i < 4; i++) {
               String fileName = files[i].getName ();
               if (files.length != 4) {
                  System.out.println ("Select all 4 necessary files.");
                  break;
               }
               // loads mandible
               if (fileName.toLowerCase ().contains ("mandible")) {
                  loadMandible (files[i]);
               }
               // loads fibula/donor
               else if (fileName.toLowerCase ().contains ("donor")
               || fileName.toLowerCase ().contains ("scapula")
               || fileName.toLowerCase ().contains ("fibula")) {

                  loadDonor (files[i]);
               }
               // Loads framemarkers for plating (Artisynth)
               else if (fileName.toLowerCase ().contains ("plate")
               || fileName.toLowerCase ().contains ("fiducial")) {
                  File platefile = files[i];
                  loadFramemarkers (platefile, false);

               }

               // Loads plane values (Artisynth)
               else if (fileName.toLowerCase ().contains ("plane")) {
                  setPlaneFile = files[i];
                  slicerPlanes = false;
                  blenderPlanes = false;
                  createPlanes (root, model);
               }
               else
                  continue;

            }
         }

         else {
            System.out.println ("Open command cancelled by user.");
         }
      }

      // Loads Mandible, Donor, Fiducials, and plane values all at once
      // for Slicer
      else if (loads.getSelectedIndex () == 2) {

         fileChooser.setMultiSelectionEnabled (true);
         int returnVal = fileChooser.showOpenDialog (fileChooser);

         if (returnVal == JFileChooser.APPROVE_OPTION) {
            File[] files = fileChooser.getSelectedFiles ();

            for (int i = 0; i < 4; i++) {
               String fileName = files[i].getName ();
               if (files.length != 4) {
                  System.out.println ("Select all 4 necessary files.");
                  break;
               }
               // loads mandible
               if (fileName.toLowerCase ().contains ("mandible")) {
                  loadMandible (files[i]);
               }
               // loads fibula/donor
               else if (fileName.toLowerCase ().contains ("donor")
               || fileName.toLowerCase ().contains ("scapula")
               || fileName.toLowerCase ().contains ("fibula")) {

                  loadDonor (files[i]);
               }
               // Loads framemarkers for plating (Slicer)
               else if (fileName.toLowerCase ().contains ("plate")
               || fileName.toLowerCase ().contains ("fiducial")) {
                  File platefile = files[i];
                  loadFramemarkers (platefile, true);
               }

               // Loads plane values (Slicer)
               else if (fileName.toLowerCase ().contains ("plane")) {
                  setPlaneFile = files[i];
                  slicerPlanes = true;
                  blenderPlanes = false;
                  createPlanes (root, model);
               }
               else
                  continue;

            }
         }

         else {
            System.out.println ("Open command cancelled by user.");
         }
      }

      else if (loads.getSelectedIndex () == 3) {
         int returnVal = fileChooser.showOpenDialog (fileChooser);

         if (returnVal == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile ();
            loadMandible (file);

         }
         else {
            System.out.println ("Open command cancelled by user.");
         }

      }
      // Loads donor
      else if (loads.getSelectedIndex () == 4) {
         int returnVal = fileChooser.showOpenDialog (fileChooser);

         if (returnVal == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile ();
            loadDonor (file);
         }
         else {
            System.out.println ("Open command cancelled by user.");
         }
      }

      // Loads framemarkers for plating (Artisynth)
      else if (loads.getSelectedIndex () == 5) {
         int returnVal = fileChooser.showOpenDialog (fileChooser);

         if (returnVal == JFileChooser.APPROVE_OPTION) {
            File platefile = fileChooser.getSelectedFile ();
            loadFramemarkers (platefile, false);
         }
         else {
            System.out.println ("Open command cancelled by user.");
         }
      }
      // Loads framemarkers for plating exported by Slicer
      else if (loads.getSelectedIndex () == 6) {
         int returnVal = fileChooser.showOpenDialog (fileChooser);

         if (returnVal == JFileChooser.APPROVE_OPTION) {
            File platefile = fileChooser.getSelectedFile ();
            loadFramemarkers (platefile, true);
         }
      }

      // Loads plane values (Artisynth)
      else if (loads.getSelectedIndex () == 7) {
         int returnVal = fileChooser.showOpenDialog (fileChooser);

         if (returnVal == JFileChooser.APPROVE_OPTION) {
            setPlaneFile = fileChooser.getSelectedFile ();
            slicerPlanes = false;
            blenderPlanes = false;
            createPlanes (root, model);
         }
         else {
            System.out.println ("Open command cancelled by user.");
         }
      }
      // Load plane values exported by Slicer
      else if (loads.getSelectedIndex () == 8) {
         int returnVal = fileChooser.showOpenDialog (fileChooser);

         if (returnVal == JFileChooser.APPROVE_OPTION) {
            setPlaneFile = fileChooser.getSelectedFile ();
            slicerPlanes = true;
            blenderPlanes = false;
            createPlanes (root, model);
         }
         else {
            System.out.println ("Open command cancelled by user.");
         }
      }
      // Load plane values exported by Blender
      else if (loads.getSelectedIndex () == 9) {
         int returnVal = fileChooser.showOpenDialog (fileChooser);
         if (returnVal == JFileChooser.APPROVE_OPTION) {
            setPlaneFile = fileChooser.getSelectedFile ();
            blenderPlanes = true;
            slicerPlanes = true;
            createPlanes (root, model);
         }
         else {
            System.out.println ("Open command cancelled by user.");
         }
      }

      // load Dental Implants
      else if (loads.getSelectedIndex () == 10) {
         int returnVal = fileChooser.showOpenDialog (fileChooser);
         if (returnVal == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile ();
            // LoadingMesh
            PolygonalMesh dentalImplantsMesh =
               meshHelper.readMesh (file.getAbsolutePath (), file.getName ());
            AffineTransform3d t = new AffineTransform3d ();
            t.setTranslation (root.TranslateToCentroidMandible);
            dentalImplantsMesh.transform (t);
            RigidBody dentalImplantsMeshBody =
               new RigidBody ("Dental Implants");
            dentalImplantsMeshBody.setSurfaceMesh (dentalImplantsMesh);
            // mechModel.addRigidBody (dentalImplantsMeshBody);
            root.getMainViewer ().autoFit ();

            //
            PolygonalMesh[] individualImplants =
               dentalImplantsMesh.partitionIntoConnectedMeshes ();
            for (int i = 0; i < individualImplants.length; i++) {
               root.implantsSeperated.add (individualImplants[i]);
               if (i < 3) {
                  meshHelper
                     .setMeshColour (
                        root.implantsSeperated.get (i), i / 3.f, 0.f, 0.f);
               }
               else if (i < 6) {
                  meshHelper
                     .setMeshColour (
                        root.implantsSeperated.get (i), 0.f, 0.f, i / 3.f);
               }
               else {
                  meshHelper
                     .setMeshColour (
                        root.implantsSeperated.get (i), 0.f, i / 3.f, 0.f);
               }
               FixedMeshBody implantBody =
                  new FixedMeshBody (
                     "implants" + Integer.toString (i),
                     root.implantsSeperated.get (i));
               model.addMeshBody (implantBody);
            }
         }
         else {
            System.out.println ("Open command cancelled by user.");
         }
      }

      // load maxilla
      else if (loads.getSelectedIndex () == 11) {
         int returnVal = fileChooser.showOpenDialog (fileChooser);

         if (returnVal == JFileChooser.APPROVE_OPTION) {

            File file = fileChooser.getSelectedFile ();
            // LoadingMesh
            PolygonalMesh maxillaMesh =
               meshHelper.readMesh (file.getAbsolutePath (), file.getName ());
            int remove = maxillaMesh.removeDisconnectedVertices ();
            RigidBody maxillaMeshBody = new RigidBody ("Maxilla");
            maxillaMeshBody.setSurfaceMesh (maxillaMesh);
            Assist
               .attachVisiblityToPanel (
                  maxillaMeshBody, root.getControlPanels ().get ("Visibility"));
            Point3d centroid = new Point3d ();
            maxillaMesh.computeCentroid (centroid);

            Vector3d maxillaCentroid = new Vector3d ();
            maxillaMesh.computeCentroid (maxillaCentroid);
            System.out.println ("Max centroid is " + maxillaCentroid);
            maxillaMesh.translateToCentroid ();

//            TODO: Fix this so that the maxilla is aligned with mandible
//            maxillaCentroid.sub (root.mandibleCentroidOriginal);

            maxillaMesh.translate (maxillaCentroid);
            maxillaMesh.computeCentroid (maxillaCentroid);

            model.addRigidBody (maxillaMeshBody);
            root.getMainViewer ().autoFit ();
         }
      }
      
      //load postop model (remember to register beforehand)
      else if (loads.getSelectedIndex () == 12) {
         int returnVal = fileChooser.showOpenDialog (fileChooser);

         if (returnVal == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile ();
            // LoadingMesh
            PolygonalMesh postopMesh =
               meshHelper.readMesh (file.getAbsolutePath (), file.getName ());
            int remove = postopMesh.removeDisconnectedVertices ();
            
            FixedMeshBody postopMeshBody = 
               Assist.FileToFixedMeshBody (file, "Postop Mandible", true);
            
            PolygonalMesh translated = (PolygonalMesh) postopMeshBody.getMesh ();     
                        
            RigidBody postopBody = new RigidBody ("Postop Mandible");
            postopBody.setSurfaceMesh (translated);
            
            model.addRigidBody (postopBody);
            root.getMainViewer ().autoFit ();
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
      if (root.donorIsScapulaCheckBox.isSelected ()) {
         // what does this do??? FIgure it out later
         // root.DonorDistanceProx.setValue (50);
         // root.DonorDistanceDis.setValue (0.01);
         // root.multiplier.setValue (0.5);
         //
         Vector3d mandibleCentroid = new Vector3d ();
//         TODO: Fix this line
         model.rigidBodies ().get ("Mandible").getSurfaceMesh ().computeCentroid (mandibleCentroid);
         RigidTransform3d transform = new maspack.matrix.RigidTransform3d ();
         transform.setTranslation (mandibleCentroid);
         root.scapulaTrimPlane = MeshFactory.createPlane (250, 90, 10, 10);
         FixedMeshBody scapulaTrimPlaneBody =
            new FixedMeshBody ("ScapulaTrimPlane", root.scapulaTrimPlane);
         model.addMeshBody (scapulaTrimPlaneBody);
         scapulaTrimPlaneBody.transformGeometry (transform);
      }
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
      Vector3d mandibleCentroid = new Vector3d ();
      Vector3d normalPlane1 = null;
      Vector3d normalPlane2 = null;
      Point3d centerPlane1 = null;
      Point3d centerPlane2 = null;
      AxisAngle nP1 = null;
      AxisAngle nP2 = null;
      Assist.GetMesh (mechModel, "Mandible").computeCentroid (mandibleCentroid);

      PolygonalMesh plane1Mesh = MeshFactory.createPlane (90, 85, 10, 10);

      RigidTransform3d transform = new RigidTransform3d ();

      // MeshPlane is created at (0,0,0)
      transform.setTranslation (mandibleCentroid);
      plane1Mesh.transform (transform);

      PolygonalMesh plane2Mesh = plane1Mesh.copy ();

      FixedMeshBody plane1MeshBody = new FixedMeshBody ("Plane1", plane1Mesh);
      plane1Mesh.translateToCentroid ();
      FixedMeshBody plane2MeshBody = new FixedMeshBody ("Plane2", plane2Mesh);
      plane2Mesh.translateToCentroid ();

      mechModel.addMeshBody (plane1MeshBody);
      mechModel.addMeshBody (plane2MeshBody);
      Assist
         .attachVisiblityToPanel (
            plane1MeshBody, root.getControlPanels ().get ("Visibility"));
      Assist
         .attachVisiblityToPanel (
            plane2MeshBody, root.getControlPanels ().get ("Visibility"));
      // Setting planes' orientation according to loaded plane values
      if (setPlaneFile != null) {
         System.out.println ("Setting plane according to input file");
         FileInputStream inputPlane = null;
         try {
            inputPlane = new FileInputStream (setPlaneFile);
            DataInputStream in = new DataInputStream (inputPlane);
            BufferedReader br = new BufferedReader (new InputStreamReader (in));
            String strLine;
            int i = 0;
            double el1 = 0.0;
            double el2 = 0.0;
            double el3 = 0.0;
            double el4 = 0.0;
            if (!blenderPlanes) {
               while ((strLine = br.readLine ()) != null) {
                  String[] tokens = strLine.split (" ");
                  el1 = Double.parseDouble (tokens[0]);
                  el2 = Double.parseDouble (tokens[1]);
                  el3 = Double.parseDouble (tokens[2]);
                  if (i == 0) {
                     normalPlane1 = new Vector3d (el1, el2, el3);
                  }
                  else if (i == 1) {
                     centerPlane1 = new Point3d (el1, el2, el3);
                  }
                  else if (i == 2) {
                     normalPlane2 = new Vector3d (el1, el2, el3);
                  }
                  else if (i == 3) {
                     centerPlane2 = new Point3d (el1, el2, el3);
                  }
                  i++;
               }
            }
            else {
               while ((strLine = br.readLine ()) != null) {
                  String[] tokens = strLine.split (" ");
                  el1 = Double.parseDouble (tokens[0]);
                  el2 = Double.parseDouble (tokens[1]);
                  el3 = Double.parseDouble (tokens[2]);
                  if (i % 2 == 0) {
                     el4 = Double.parseDouble (tokens[3]);
                     el4 = (el4 / 180.0) * Math.PI;
                  }
                  if (i == 0) {
                     nP1 = new AxisAngle (el1, el2, el3, el4);
                  }
                  else if (i == 1) {
                     centerPlane1 = new Point3d (el1, el2, el3);
                  }
                  else if (i == 2) {
                     nP2 = new AxisAngle (el1, el2, el3, el4);
                  }
                  else if (i == 3) {
                     centerPlane2 = new Point3d (el1, el2, el3);
                  }
                  i++;
               }
            }

            Vector3d currentPlane1Normal = plane1Mesh.getNormal (0);
            Vector3d currentPlane2Normal = plane2Mesh.getNormal (0);
            RigidTransform3d pose1 = plane1MeshBody.getPose ();
            RigidTransform3d pose2 = plane2MeshBody.getPose ();
            currentPlane1Normal.transform (pose1);
            currentPlane2Normal.transform (pose2);

            RotationMatrix3d rotatePlane1 =
               meshHelper.rotatePlane (currentPlane1Normal, normalPlane1);
            RotationMatrix3d rotatePlane2 =
               meshHelper.rotatePlane (currentPlane2Normal, normalPlane2);

            AffineTransform3d t1 = new AffineTransform3d ();
            AffineTransform3d t2 = new AffineTransform3d ();
            t1.setRotation (rotatePlane1);
            t2.setRotation (rotatePlane2);

            if (blenderPlanes) {
               t1.setRotation (nP1);
               t2.setRotation (nP2);
            }
            // ????
            plane1MeshBody.transformGeometry (t1);
            plane2MeshBody.transformGeometry (t2);

            if (slicerPlanes) {
               centerPlane1.add (root.TranslateToCentroidMandible);
               centerPlane2.add (root.TranslateToCentroidMandible);
            }
            Assist.setPlaneOrigin (plane1MeshBody, centerPlane1);
            Assist.setPlaneOrigin (plane2MeshBody, centerPlane2);
            // meshHelper.setPlaneOrigin (plane1MeshBody, centerPlane1);
            // meshHelper.setPlaneOrigin (plane2MeshBody, centerPlane2);

            if (blenderPlanes) {
               normalPlane1 = new Vector3d (plane1Mesh.getNormal (0));
               normalPlane1.transform (plane1MeshBody.getPose ());
               normalPlane2 = new Vector3d (plane2Mesh.getNormal (0));
               normalPlane2.transform (plane2MeshBody.getPose ());
            }

            // rerender ();

         }
         catch (IOException e) {
            System.out.println ("Exception");
         }
      }
   }
}