package artisynth.istar.Prisman;

import java.io.*;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.NumberFormat;
import java.awt.Color;
import java.awt.GridLayout;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSplitPane;
import javax.swing.JTextField;

import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.renderables.EditablePolygonalMeshComp;
import artisynth.core.workspace.RootModel;
import artisynth.demos.test.IntersectionTester;
import artisynth.core.gui.*;
import artisynth.core.util.*;
import maspack.geometry.*;
import maspack.geometry.io.GenericMeshWriter;
import maspack.geometry.io.StlReader;
import maspack.interpolation.Interpolation;
import maspack.interpolation.Interpolation.Order;
import maspack.interpolation.NumericList;
import maspack.interpolation.NumericListKnot;
import maspack.matrix.*;
import maspack.util.*;
import maspack.render.*;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.Shading;
import maspack.widgets.ExpandablePropertyPanel;
import maspack.widgets.LabeledComponentBase;
import maspack.widgets.LabeledToggleButton;
import maspack.collision.*;
import maspack.collision.SurfaceMeshIntersector.CSG;

import java.awt.event.ActionEvent;
import maspack.collision.SurfaceMeshIntersectorTest;
import maspack.collision.SurfaceMeshIntersector;
import artisynth.core.gui.editorManager.Command;
import artisynth.core.gui.editorManager.UndoManager;
import artisynth.core.driver.Main;
import artisynth.istar.Prisman.undo.ClipMandCommand;
import artisynth.core.util.ArtisynthPath;

public class Measurements extends RootModel {

   public static String rbpath =
      ArtisynthPath
         .getHomeRelativePath ("src/maspack/geometry/sampleData/", ".");
   Vector3d maxillaCentroid = new Vector3d();


   PolygonalMesh maxillaMesh, objectMesh = null;

   PolygonalMesh masterGuideMesh = null;


   RigidBody maxillaMeshBody, objectMeshBody;
   
   FixedMeshBody rdpMeshBody;
   RigidBody clippedDonorMeshBody;
   PolygonalMesh DonorGuide;
   FixedMeshBody DonorGuideBody;
   PolygonalMesh maxillareconstructed;
   FixedMeshBody reconstruction;
   FixedMeshBody Mastermaxilla;
   FixedMeshBody MasterGuide;
   PolygonalMesh squareframe;
   PolygonalMesh metalinsert;
   PolygonalMesh squareframelong;
   PolygonalMesh metalinsertlong;

   PolygonalMesh planeMesh, reversePlaneMesh;
   RigidBody planeMeshBody, reversePlaneMeshBody;
   PolygonalMesh maxillaMesh1, maxillaMesh2;
   RigidBody maxillaMesh1Body, maxillaMesh2Body;

   NumericList plateNumericList;
   NumericList simpList;
   RigidBody tempMeshBody;
   List<Point3d> spoints = new ArrayList<Point3d> ();
   Point3d topPoint = new Point3d ();
   List<Point3d> frameMarkerPos = new ArrayList<Point3d> ();
   List<FrameMarker> fm = new ArrayList<FrameMarker> ();
   Vector3d TranslateToCentroidmaxilla = new Vector3d ();

   PolygonalMeshRenderer meshRenderer;

   JButton measureButton, clearListButton, cutMaxillaButton, resetMaxillaButton;

   JComboBox exports, loads;
   JFileChooser fileChooser;

   ControlPanel myControlPanel;
   ControlPanel myControlPanel2;
   MechModel mechModel;

   HelperMathFunctions mathHelper = new HelperMathFunctions ();
   HelperMeshFunctions meshHelper = new HelperMeshFunctions ();

   private void addControlPanel () {

      // Buttons
      myControlPanel = new ControlPanel ("Control Panel", "");
      
      cutMaxillaButton = new JButton ("Cut Maxilla");
      cutMaxillaButton.addActionListener (new cutMaxillaButtonClicked ());
      
      resetMaxillaButton = new JButton ("Reset Maxilla");
      resetMaxillaButton.addActionListener (new resetMaxillaButtonClicked ());
      
      measureButton = new JButton ("Measure");
      measureButton.addActionListener (new measureButtonClicked ());

      clearListButton = new JButton ("Clear FrameMarkers");
      clearListButton.addActionListener (new clearListButtonClicked ());

      String[] loadList =
         { "<None>", "Maxilla + Recon", "Object" };
      loads = new JComboBox (loadList);
      loads.setSelectedIndex (0);
      loads.addActionListener (new LoadFiles ());
      JSplitPane loadPane = new JSplitPane ();
      JLabel loadPaneLabel = new JLabel ("Load: ");
      loadPane.setLeftComponent (loadPaneLabel);
      loadPane.setRightComponent (loads);

      // File Chooser
      fileChooser = new JFileChooser ();

      myControlPanel.addWidget (loadPane);
      myControlPanel.addWidget (cutMaxillaButton);
      myControlPanel.addWidget (resetMaxillaButton);
      myControlPanel.addWidget (measureButton);
      myControlPanel.addWidget (clearListButton);

      addControlPanel (myControlPanel);
   }

   public class LoadFiles extends AbstractAction {
      public LoadFiles () {
         putValue (NAME, "Load Stl Files");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         if (loads.getSelectedIndex () == 1) {               
               fileChooser.setMultiSelectionEnabled (true);
               int returnVal = fileChooser.showOpenDialog (fileChooser);

               if (returnVal == JFileChooser.APPROVE_OPTION) {
                  File[] files = fileChooser.getSelectedFiles ();

                  for (int i = 0; i < 2; i++) {
                     String fileName = files[i].getName ();
                     if (files.length != 2) {
                        System.out.println ("Select both necessary files.");
                        break;
                     }
                     // loads maxilla
                     if (fileName.toLowerCase ().contains ("maxilla")) {
                        loadMaxilla(files[i]);
                     }
                     // loads fibula/scapula
                     else {

                        loadObject(files[i], "Recon" + i);

                     }
                  }
                  
               
            }
            else {
               System.out.println ("Open command cancelled by user.");
            }

         } else {
            int returnVal = fileChooser.showOpenDialog (fileChooser);
            
            if (returnVal == JFileChooser.APPROVE_OPTION) {
               File file = fileChooser.getSelectedFile ();
               if(file.getName ().toLowerCase ().contains ("maxilla")) {
                  loadMaxilla(file);
               } else {
                  loadObject(file, "Object For Measurement");
               }
               
            }
         }
      }
      
      public void loadObject(File f, String name) {
         objectMesh =
            meshHelper
               .readMesh (
                  f.getAbsolutePath (), f.getName ());
         int remove = objectMesh.removeDisconnectedVertices ();
         objectMeshBody = new RigidBody (name);
         objectMeshBody.setMesh (objectMesh);
         mechModel.addRigidBody (objectMeshBody);
         getMainViewer ().autoFit ();
         rerender ();
      }
      
      public void loadMaxilla(File f) {
         maxillaMesh =
            meshHelper
               .readMesh (f.getAbsolutePath (), f.getName ());
         int remove = maxillaMesh.removeDisconnectedVertices ();
         maxillaMeshBody = new RigidBody ("Maxilla");
         maxillaMeshBody.setMesh (maxillaMesh);
         Point3d centroid = new Point3d ();
         maxillaMesh.computeCentroid (centroid);
         maxillaMesh.computeCentroid (maxillaCentroid);
         TranslateToCentroidmaxilla = new Vector3d (centroid).negate ();
         maxillaMesh.translateToCentroid ();
         mechModel.addRigidBody (maxillaMeshBody);
         getMainViewer ().autoFit ();
         
         planeMesh = MeshFactory.createPlane (180, 180);
         planeMesh.translateToCentroid();
         planeMeshBody = new RigidBody("Plane");
         planeMeshBody.setMesh (planeMesh);
         mechModel.addRigidBody (planeMeshBody);
         rerender();
      }
   }
   
   public class cutMaxillaButtonClicked extends AbstractAction {
      // Cut maxilla in half defined by planeMesh
      public cutMaxillaButtonClicked () {
         putValue (NAME, "Cut Maxilla Button Clicked");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         mechModel.removeRigidBody (maxillaMesh1Body);
         mechModel.removeRigidBody (maxillaMesh2Body);
         
         SurfaceMeshIntersector intersector =
            new maspack.collision.SurfaceMeshIntersector ();
         maxillaMesh1 = intersector.findDifference01 (maxillaMesh, planeMesh);
         
         Vector3d origPlaneCentroid = new Vector3d();
         planeMesh.computeCentroid (origPlaneCentroid);
         
         reversePlaneMesh = MeshFactory.createPlane (90, 90);
         
         Point3d centroidPlane = new Point3d();
         centroidPlane.x = origPlaneCentroid.x;
         centroidPlane.y = origPlaneCentroid.y;
         centroidPlane.z = origPlaneCentroid.z;
         meshHelper.createPlane (
            reversePlaneMesh.getNormal (0), 
            planeMesh.getNormal (0).negate (), 
            centroidPlane, 
            planeMeshBody.getPose (), 
            reversePlaneMesh);
         
         maxillaMesh2 = intersector.findDifference01 (maxillaMesh, reversePlaneMesh);
         
         reversePlaneMeshBody = new RigidBody("Reverse Plane");
         reversePlaneMeshBody.setMesh (reversePlaneMesh);
         
         maxillaMesh2Body = new RigidBody("Maxilla Mesh 2");
         maxillaMesh2Body.setMesh (maxillaMesh2);
         
         maxillaMesh1Body = new RigidBody("Maxilla Mesh 1");
         maxillaMesh1Body.setMesh (maxillaMesh1);
         
         mechModel.addRigidBody (maxillaMesh1Body);
         mechModel.addRigidBody (maxillaMesh2Body);
         maxillaMeshBody.getRenderProps ().setVisible (false);
         maxillaMesh2Body.getRenderProps ().setVisible (false);
         rerender();
      }
   }
   
   public class resetMaxillaButtonClicked extends AbstractAction {
      // Cut maxilla in half defined by planeMesh
      public resetMaxillaButtonClicked () {
         putValue (NAME, "Reset Maxilla Button Clicked");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         maxillaMeshBody.getRenderProps ().setVisible (true);
         maxillaMesh1Body.getRenderProps ().setVisible (false);
         
      }
   }

   public class measureButtonClicked extends AbstractAction {
      // Aligns the Donor to the maxilla by translating the Donor centroid to
      // the maxilla centroid
      public measureButtonClicked () {
         putValue (NAME, "Align Button Clicked");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         RenderableComponentList<FrameMarker> frameMarkers = mechModel.frameMarkers ();
         for (int i=0; i<frameMarkers.size ()-1; i++) {
            System.out.println ("Distance between poinst " + i + " and " + (i+1) + " is " + frameMarkers.get (i).distance (frameMarkers.get (i+1)));
         }
      }
   }

   public class clearListButtonClicked extends AbstractAction {
      public clearListButtonClicked () {
         putValue (NAME, "Clear list");
      }

      @Override
      public void actionPerformed (ActionEvent evt) {
         mechModel.clearFrameMarkers ();
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
      //addHideShowControlPanel ();
      fm.clear ();
      frameMarkerPos.clear ();
      String homedir = ArtisynthPath.getHomeDir ();
      File pathHome = new File (homedir);
      String homeParent = pathHome.getParentFile ().getAbsolutePath ();

      System.out.println (homeParent);

      // String path =
      // homeParent
      // +
      // "/artisynth_projects/src/artisynth/models/Prisman/stl/Mastermaxilla.stl";
      // mastermaxillaMesh = meshHelper.readMesh (path, "Mastermaxilla.stl");
      String path =
         homeParent
         + "/artisynth_projects/src/artisynth/models/Prisman/stl/MasterInnerGuide.stl";
      masterGuideMesh = meshHelper.readMesh (path, "MasterInnerGuide.stl");
      path =
         homeParent
         + "/artisynth_projects/src/artisynth/models/Prisman/stl/squareFrame30.stl";
      squareframe = meshHelper.readMesh (path, "squareFrame30.stl");
      path =
         homeParent
         + "/artisynth_projects/src/artisynth/models/Prisman/stl/bladeGuideUnion30.stl";
      metalinsert = meshHelper.readMesh (path, "bladeGuideUnion30.stl");
      path =
         homeParent
         + "/artisynth_projects/src/artisynth/models/Prisman/stl/squareFrame.stl";
      squareframelong = meshHelper.readMesh (path, "bladeGuideUnion30.stl");
      path =
         homeParent
         + "/artisynth_projects/src/artisynth/models/Prisman/stl/bladeGuideUnion.stl";
      metalinsertlong = meshHelper.readMesh (path, "bladeGuideUnion.stl");

   }
}
