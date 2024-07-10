package artisynth.istar.Prisman;

import java.io.*;
import java.util.ArrayList;
import java.awt.Color;
import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JSplitPane;
import artisynth.core.mechmodels.*;
import artisynth.core.workspace.RootModel;
import artisynth.istar.Prisman.undo.AddImplantsCommand;
import artisynth.core.driver.Main;
import artisynth.core.gui.*;
import artisynth.core.gui.editorManager.UndoManager;
import maspack.geometry.*;
import maspack.geometry.io.GenericMeshWriter;
import maspack.matrix.*;
import maspack.render.*;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.solvers.PardisoSolver;

import java.awt.event.ActionEvent;
import artisynth.core.util.ArtisynthPath;


public class DentalImplants extends RootModel{

   public static String rbpath =
   ArtisynthPath
      .getHomeRelativePath ("src/maspack/geometry/sampleData/", ".");
   
   PolygonalMesh mandibleMesh = null;
   RigidBody mandibleMeshBody;
   ArrayList<PolygonalMesh> implantMeshes = new ArrayList<PolygonalMesh> ();
   ArrayList<FixedMeshBody> implantBodies = new ArrayList<FixedMeshBody> ();
   
   
   JComboBox exports, loads;
   JFileChooser fileChooser;

   Vector3d TranslateToCentroidMandible = new Vector3d ();
   
   PolygonalMeshRenderer meshRenderer;

   JButton addImplantButton, saveImplantsButton;
   
   ControlPanel myControlPanel;
   MechModel mechModel;

   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();
   

   private void addControlPanel () { 
      myControlPanel = new ControlPanel ("Control Panel", "");
      
      String[] loadList =
      { "<None>", "Mandible" };
      loads = new JComboBox (loadList);
      loads.setSelectedIndex (0);
      loads.addActionListener (new LoadFiles ());
      JSplitPane loadPane = new JSplitPane ();
      JLabel loadPaneLabel = new JLabel ("Load: ");
      loadPane.setLeftComponent (loadPaneLabel);
      loadPane.setRightComponent (loads);
      
      addImplantButton = new JButton("Add Implant");
      addImplantButton.addActionListener (new AddImplantButtonClicked ());
      
      saveImplantsButton = new JButton("Save Implants");
      saveImplantsButton.addActionListener (new SaveImplantsClicked ());
      
      fileChooser = new JFileChooser ();
      myControlPanel.addWidget (loadPane);
      myControlPanel.addWidget (addImplantButton);
      myControlPanel.addWidget (saveImplantsButton);
      addControlPanel (myControlPanel);
   
   }

   public class LoadFiles extends AbstractAction {
      public LoadFiles () {
         putValue (NAME, "Load Stl Files");
      }
   
      @Override
      public void actionPerformed (ActionEvent evt) {
         // Loads mandible
         if (loads.getSelectedIndex () == 1) {
            int returnVal = fileChooser.showOpenDialog (fileChooser);
   
            if (returnVal == JFileChooser.APPROVE_OPTION) {
               File file = fileChooser.getSelectedFile ();
               // LoadingMesh
               mandibleMesh =
                  meshHelper
                     .readMesh (file.getAbsolutePath (), file.getName ());
               mandibleMesh.removeDisconnectedVertices ();
               mandibleMeshBody = new RigidBody ("Mandible");
               mandibleMeshBody.setMesh (mandibleMesh);
               Point3d centroid = new Point3d ();
               mandibleMesh.computeCentroid (centroid);
               TranslateToCentroidMandible = new Vector3d (centroid).negate ();
               mandibleMesh.translateToCentroid ();
               mechModel.addRigidBody (mandibleMeshBody);
               getMainViewer ().autoFit ();
            }
            else {
               System.out.println ("Open command cancelled by user.");
            }
   
         }
        
            else {
               System.out.println ("Open command cancelled by user.");
            }
         }
      }
   
   public class AddImplantButtonClicked extends AbstractAction {
      public AddImplantButtonClicked() {
         putValue (NAME, "Add Dental Implant");
      }
      
      @Override
      public void actionPerformed (ActionEvent evt) {

       implantMeshes.add (MeshFactory.createCylinder (1, 60, 100));
       implantBodies.add (new FixedMeshBody( "Implant" + Integer.toString(implantMeshes.size() + 1), implantMeshes.get (implantMeshes.size() - 1)));
       
        AddImplantsCommand cmd = new AddImplantsCommand("Add Implant", implantBodies, mechModel );
        Main main = Main.getMain ();
        UndoManager undo = main.getUndoManager ();
        undo.execute (cmd);

      }
      
   }
   
   public class SaveImplantsClicked extends AbstractAction {
      public SaveImplantsClicked() {
         putValue (NAME, "SaveImplants");
      }
      
      @Override
      public void actionPerformed (ActionEvent evt) {
         implantMeshes.get (0).transform (implantBodies.get (0).getPose());
         implantBodies.get (0).setPose(new RigidTransform3d());
         PolygonalMesh combinedImplants = new PolygonalMesh(implantMeshes.get (0));
         for (int i = 1; i < implantMeshes.size (); i++) {
          implantMeshes.get (i).transform (implantBodies.get (i).getPose());
          implantBodies.get (i).setPose(new RigidTransform3d());
          combinedImplants = MeshFactory.getUnion (combinedImplants, implantMeshes.get (i)); 
       }
       
       Vector3d translate = new Vector3d (TranslateToCentroidMandible);
       AffineTransform3d t = new AffineTransform3d ();
       t.setTranslation (translate);
       combinedImplants.inverseTransform (t);
 
       try {
          System.out.println ("Saving Implants.");
         GenericMeshWriter.writeMesh ("CombinedImplants.stl", combinedImplants);
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      }
      
   }
   
   
   @Override
   public void build (String[] args) {
      mechModel = new MechModel ("msmod");
      RenderProps.setPointStyle (mechModel, PointStyle.SPHERE);
      RenderProps.setPointRadius (mechModel, 1.25);
      RenderProps.setFaceStyle (mechModel, FaceStyle.FRONT_AND_BACK);
      RenderProps.setBackColor (mechModel, Color.GREEN);
      
      
      addModel (mechModel);
      addControlPanel ();

      String homedir = ArtisynthPath.getHomeDir ();
      File pathHome = new File (homedir);
      String homeParent = pathHome.getParentFile ().getAbsolutePath ();

      System.out.println (homeParent); 

   }
}