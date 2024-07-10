 package artisynth.istar.RuiToothRemoval;
//Rui: from artisynth_models_registration
import java.awt.Color;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Hashtable;
import java.util.LinkedList;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JSeparator;
import javax.swing.JSlider;
import javax.swing.JTextArea;
import javax.swing.SwingUtilities;
import javax.swing.event.MouseInputListener;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.editorManager.EditorUtils;
import artisynth.core.gui.selectionManager.ClassFilter;
import artisynth.core.gui.selectionManager.SelectionEvent;
import artisynth.core.gui.selectionManager.SelectionListener;
import artisynth.core.gui.selectionManager.SelectionManager;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ExtensionFileFilter;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.core.femmodels.EmbeddedFem;
import artisynth.models.registration.DynamicRegistrationController;
import artisynth.models.registration.correspondences.GMMMeshCorrespondence;
import maspack.collision.IntersectionContour;
import maspack.collision.IntersectionPoint;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.ICPRegistration;
import maspack.geometry.MeshFactory;
import maspack.geometry.MeshUtilities;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.Shading;
import maspack.widgets.BooleanSelector;
import maspack.widgets.ExpandablePropertyPanel;
import maspack.widgets.GuiUtils;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;

public class CSGSegOneByOne extends RootModel {
   FemMeshComp source;
   PolygonalMesh cMesh; //contour mesh, clipmesh
   PolygonalMesh ctrDentureMesh;
   PolygonalMesh ctrBridgeMesh;
   PolygonalMesh ctrSliderMesh;
   PolygonalMesh opvMesh;
   PolygonalMesh sourcePolyMesh;
   FemMeshComp cMeshComp;
   FemMeshComp opvComp;
   RigidBody baseOP;
   Plane op;
   PolygonalMesh target;
   FixedMeshBody fixedTarget;
   FixedMeshBody fixedCMesh;
   FixedMeshBody fixedSmoothedMesh;
   FixedMeshBody fixedCtrDenture;
   FixedMeshBody fixedCtrBridge;
   FixedMeshBody fixedCtrSlider;
   FemModel3d fem;
   double opDist=0;
   Point3d targetTr = new Point3d();
   PolygonalMesh smoothedMesh;  
   static ArrayList<ArrayList<Point3d>> ctrlPnts3d = new ArrayList<ArrayList<Point3d>>();
   JProgressBar progress;
   double manualEditRadius = 10;
   double radiusScaling =1;
   PolygonalMesh sphere = MeshFactory.createSphere(manualEditRadius,48);
   FixedMeshBody sphereBody = new FixedMeshBody();   
   Point selectedPnt = null;
   Point3d currPos = new Point3d();
   JTextArea instructions;
   BooleanSelector renderDiffMesh;
   BooleanSelector smoothing;
   BooleanSelector oneSidedControl;
   BooleanSelector clipToSurface;  
   BooleanSelector initialContour;
   BooleanSelector bridgeLevel;
   BooleanSelector dentureLevel;
   BooleanSelector opVisible;
   BooleanSelector renderRange;
   BooleanSelector showSliderMesh;
   JSlider heightSlider;
   JButton ctrlPntReassignment;
   JButton allPntSelection;
   boolean isCtrlPntsSet = false; //false
   double runTime = 1;  //1 for 10k-20k-vertex meshes
   double fDensity = 0.0001; //0.0001 for 10k-20k-vertex meshes
   double fScaling = 1e6;  //1e6 for 10k-20k-vertex meshes
   Intersector myIntersector; 
   Intersector intersectorBridge; 
   Intersector intersectorDenture; 
   ArrayList<IntersectionContour> myContours = new ArrayList<IntersectionContour>();
   ArrayList<Point> controlPoints = new  ArrayList<Point>();
   ArrayList<Point3d> originalPos = new  ArrayList<Point3d>();
   int selectedIdx=-1;
   ArrayList<GeoVertex> vertDistTemp = new ArrayList<GeoVertex>();
   ArrayList<ArrayList<GeoVertex>> vertDistTempList = new ArrayList<ArrayList<GeoVertex>>();
   ArrayList<Point> pntDistTemp = new ArrayList<Point>();
   MechModel mech = new MechModel("mech");
   ControlPanel panel;
   private static Color violet = new Color(0.8f, 0.8f, 1.0f);
   private static Color bleu = new Color(0.26f, 0.52f, 0.95f);
   long veryStartTime=System.nanoTime(); 
   long startOfManualTime=System.nanoTime();
   long startOfManualTime2=System.nanoTime();
   int selectionNum=0;
   ArrayList<Point> mcPList = new ArrayList<Point>();
   String modelName;
   JLabel distanceLabel;
   double currentHeight;
   String sourceMeshName = "mandible_3kv.stl";  // mandible14.stl
   
   public static PropertyList myProps =
   new PropertyList (CSGSegOneByOne.class, RootModel.class);
   static {
      myProps.addReadOnly ("controlRadius", "the range a point can control");
      }
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
  
   @Override
   public void build(String[] args) throws IOException {                         
      super.build(args);     
      addModel(mech);     
      mech.setGravity(0, 0, 0);  // turn off gravity for registration
     // setAdaptiveStepping(false);  //false by default      
      String meshDirectory = ArtisynthPath.getSrcRelativePath(this, "mandible/");
         
      String cMeshName = "16clip.stl"; //   
      String opvName = "OP.stl"; //occlusal plane vertices
         
   // load source mesh (template with teeth)           
      sourcePolyMesh = GenericModel.loadGeometry(meshDirectory, sourceMeshName);
      Point3d tr = new Point3d();
      tr = transformMeshOrigin(sourcePolyMesh); 
      
   // create a FEM for source mesh (template with teeth)
      fem = createVoxelizedFem(null, sourcePolyMesh, 15);
      fem.setName("fem");
      fem.setDensity(fDensity); 
      source = fem.addMesh("source", sourcePolyMesh);  
      fem.setMaterial(new LinearMaterial(100, 0.3));    
      mech.addModel(fem);
        
    // load contour mesh and put it in the FEM
      cMesh = GenericModel.loadGeometry(meshDirectory, cMeshName); 
      cMesh.translate (tr);      
      cMeshComp = fem.addMesh("cMesh", cMesh);  
      fixedCMesh = new FixedMeshBody("FixedCMesh", cMesh);
      mech.addMeshBody (fixedCMesh);
      
      opvMesh = GenericModel.loadGeometry(meshDirectory, opvName); 
      opvMesh.translate (tr);      
      opvComp = fem.addMesh("opv", opvMesh);  

      sphereBody = new FixedMeshBody("Sphere",sphere);
      mech.addMeshBody (sphereBody);
      
      // Only objects of point class are selectable
      ClassFilter myClassFilter = new ClassFilter (Point.class);     
      Main.getMain ().getSelectionManager ().addFilter (myClassFilter);
      Main.getMain ().getSelectionManager ().filterSelections (myClassFilter);
       
      RenderProps.setVisible (fem.getNodes (), false);
      RenderProps.setVisible (fem.getElements (), false);
      RenderProps.setVisible (source, false);     
      RenderProps.setFaceColor (source, Color.pink);
      RenderProps.setVisible (cMeshComp, false); 
      RenderProps.setAlpha (fixedCMesh, 0.2); //0.2
      RenderProps.setDrawEdges (fixedCMesh, true);
      RenderProps.setEdgeColor (fixedCMesh, Color.cyan);
//      RenderProps.setAlpha (fixedCMesh, 0.2); //0.2
//      RenderProps.setFaceStyle (fixedCMesh,FaceStyle.FRONT_AND_BACK);
//      RenderProps.setFaceColor (fixedCMesh, Color.cyan);
      RenderProps.setVisible (opvComp, false); 
     
      RenderProps.setVisible (sphereBody, false);
      RenderProps.setAlpha (sphereBody, 0.2);
      RenderProps.setFaceColor (sphereBody, violet);
      RenderProps.setShading (sphereBody, Shading.SMOOTH);

      Main.getMain ().setTimelineVisible (false);
      panel = new ControlPanel("Manual editing"); 
      panel.addLabel ("Messages"); 
      
      instructions = new JTextArea ("Ready to load a new model. ",  4, 1); 
      instructions.setForeground (bleu);
      instructions.setEditable (false);
      panel.addWidget (instructions); 
      
      progress = new JProgressBar();
      progress.setMinimum(0);
      progress.setMaximum(3);//iterSnake+1
      panel.addWidget (progress);
     
      renderDiffMesh = new BooleanSelector("Remove teeth", false);
      panel.addWidget (renderDiffMesh);
      renderDiffMesh.getLabel ().setEnabled (false);
      renderDiffMesh.getCheckBox ().setEnabled (false);
      renderDiffMesh.addValueChangeListener (new ValueChangeListener() {
         public void valueChange (ValueChangeEvent e) {
            Boolean val = (Boolean)(e.getValue());
            if (val.booleanValue()) {              
               if(myIntersector != null) {
                  isCtrlPntsSet = false;
                  myIntersector.updateDiffMesh ();
                  if(myIntersector.getDiffMesh ()!= null) {                  
                     boolean ctrIsOpen = false;
                     myContours = myIntersector.getContours ();
                     for(IntersectionContour ctr: myContours) {
                        if(ctr.isClosed ()==false) {
                           ctrIsOpen = true;
                           renderDiffMesh.setValue (false);
                           instructions.setText(modelName); 
                           instructions.append ("\n\nA contour is open.");
                           instructions.append ("\nMore manual editting is needed. ");                        
                           instructions.setForeground (Color.red); 
                           break;
                        }
                     }
                     if(ctrIsOpen ==false) {
                        RenderProps.setVisible (fixedTarget, false);
                        myIntersector.setContoursOnly (false);                   
                        myIntersector.setRenderDiffMesh (true); 
                        RenderProps.setVisible (sphereBody, false);            
                        smoothing.getLabel ().setEnabled (true);
                        smoothing.getCheckBox ().setEnabled (true);
                        oneSidedControl.getLabel ().setEnabled (false);
                        oneSidedControl.getCheckBox ().setEnabled (false);
                        clipToSurface.getLabel ().setEnabled (false);
                        clipToSurface.getCheckBox ().setEnabled (false);
                        allPntSelection.setEnabled (false);
                        ctrlPntReassignment.setEnabled (false);      
                        
                        for(Point p : controlPoints)
                           RenderProps.setVisible (p, false);  
                        RenderProps.setVisible (fixedCMesh, false); 
                        
                        instructions.setText(modelName);                                     
                        instructions.setForeground (bleu);
                        }                  
                     }
                  }
               } 
            else { // teeth back
               RenderProps.setVisible (fixedTarget, true);
               myIntersector.setContoursOnly (true);
               myIntersector.setRenderDiffMesh (false); 
               smoothing.getLabel ().setEnabled (false);
               smoothing.getCheckBox ().setEnabled (false);
               oneSidedControl.getLabel ().setEnabled (true);
               oneSidedControl.getCheckBox ().setEnabled (true);
               clipToSurface.getLabel ().setEnabled (true);
               clipToSurface.getCheckBox ().setEnabled (true);
               RenderProps.setVisible (fixedCMesh, true); 
               
               if(controlPoints.size ()==0)
                  isCtrlPntsSet = true;
               else {
                  for(Point p : controlPoints)
                     RenderProps.setVisible (p, true);                 
                  allPntSelection.setEnabled (true);
                  ctrlPntReassignment.setEnabled (true); 
               }

               instructions.setText(modelName);                
               instructions.setForeground (bleu);  
            }
            rerender();
            
         }
      });
           
      smoothing = new BooleanSelector("Smooth the cut area", false);
      panel.addWidget (smoothing);
      smoothing.getLabel ().setEnabled (false);
      smoothing.getCheckBox ().setEnabled (false);
      smoothing.addValueChangeListener (new ValueChangeListener() {
         public void valueChange (ValueChangeEvent e) {
            Boolean val = (Boolean)(e.getValue());
            if (val.booleanValue()) {
               if(myIntersector != null && myIntersector.getDiffMesh () != null) {                 
                  smoothedMesh =  myIntersector.getDiffMesh ().copy ();                 
                  MeshUtilities.closeHoles (smoothedMesh, 0);                
                  smoothedMesh.mergeCloseVertices (0.1);
                  for(int i=0; i< myIntersector.getContours ().size (); i++) {
                     CSGContourProcess.aveSmoothing(
                        myIntersector.getContours ().get (i), smoothedMesh);  
                     }                  
                  }
               instructions.setText(modelName); 
               instructions.append ("\n\nSmoothing completed!");              
               instructions.setForeground (bleu); 
               
               RenderProps.setVisible (myIntersector, false); 
               fixedSmoothedMesh = new FixedMeshBody("fixedSmoothed", smoothedMesh);
               //can't use addMeshBody, will remove the ctrlPnts somehow
               mech.add (fixedSmoothedMesh);
               RenderProps.setAlpha (fixedSmoothedMesh, 1);
               RenderProps.setFaceColor (fixedSmoothedMesh, new Color (1f, 1f, 0.8f));     
               
               renderDiffMesh.getLabel ().setEnabled (false);
               renderDiffMesh.getCheckBox ().setEnabled (false);
               } 
            else {
               RenderProps.setVisible (myIntersector, true);
               smoothedMesh=null;
               RenderProps.setVisible (fixedSmoothedMesh, false);
               mech.remove (fixedSmoothedMesh);
               renderDiffMesh.getLabel ().setEnabled (true);
               renderDiffMesh.getCheckBox ().setEnabled (true);
               
               instructions.setText(modelName);               
               instructions.setForeground (bleu);                              
            }
            
            rerender();             
            }
         });
      
      panel.addWidget(new JSeparator());
      panel.addLabel ("Point selection settings");  
//      panel.addWidget ("Point radius", mech.points(),"renderProps.pointRadius",0.5,2);
//      
      JPanel buttons = new JPanel ();
      buttons.setLayout (new GridLayout (0, 2));
      allPntSelection = new JButton("Select all");
      allPntSelection.addActionListener (this);
      allPntSelection.setEnabled (false);
      buttons.add (allPntSelection);
      
      ctrlPntReassignment = new JButton("Reassign points");
      ctrlPntReassignment.addActionListener (this);
      ctrlPntReassignment.setEnabled (false);      
      buttons.add (ctrlPntReassignment);
      panel.addWidget (buttons);
      panel.addWidget(this, "controlRadius");
      
      oneSidedControl = new BooleanSelector("One-sided control", false);
      panel.addWidget (oneSidedControl);
      oneSidedControl.getLabel ().setEnabled (false);
      oneSidedControl.getCheckBox ().setEnabled (false);   
      
      clipToSurface = new BooleanSelector("Clip on to the surface", false);
      panel.addWidget (clipToSurface);
      clipToSurface.getLabel ().setEnabled (false);
      clipToSurface.getCheckBox ().setEnabled (false);  
      
      panel.addWidget(new JSeparator());
      ExpandablePropertyPanel donorHeightPanel = new ExpandablePropertyPanel ();
      JLabel donorHeightLabel = new JLabel ("Donor bone height planning");
      donorHeightLabel.setForeground (new Color (0.4f, 0.4f, 0.8f));
      GuiUtils.setItalicFont (donorHeightLabel);
      donorHeightPanel.addExtraWidget (donorHeightLabel);
  
      distanceLabel = new JLabel();
      
      initialContour = new BooleanSelector("Initial contour", true);
      donorHeightPanel.addExtraWidget (initialContour);
      initialContour.getLabel ().setEnabled (false);
      initialContour.getCheckBox ().setEnabled (false); 
      initialContour.addValueChangeListener (new ValueChangeListener() {
         public void valueChange (ValueChangeEvent e) {
            Boolean val = (Boolean)(e.getValue());
            if (val.booleanValue()) {
               RenderProps.setVisible (fixedCMesh, true); 
               if(myIntersector != null)
                  RenderProps.setVisible (myIntersector, true); 
               renderCtrlPnts();
               }
            else{
               RenderProps.setVisible (fixedCMesh, false);
               if(myIntersector != null)
                  RenderProps.setVisible (myIntersector, false); 
               mech.clearPoints ();
               }
            }
         });
            
      opVisible = new BooleanSelector("The occlusal plane", false);
      donorHeightPanel.addExtraWidget (opVisible);
      opVisible.getLabel ().setEnabled (false);
      opVisible.getCheckBox ().setEnabled (false);
      opVisible.addValueChangeListener (new ValueChangeListener() {
         public void valueChange (ValueChangeEvent e) {
            Boolean val = (Boolean)(e.getValue());
            if (val.booleanValue()) {
               RenderProps.setVisible (baseOP, true);
               }
            else {
               RenderProps.setVisible (baseOP, false);
               }
            }
         });
      
      bridgeLevel = new BooleanSelector("Implant bridge (10 mm)", false);
      donorHeightPanel.addExtraWidget (bridgeLevel);
      bridgeLevel.getLabel ().setEnabled (false);
      bridgeLevel.getCheckBox ().setEnabled (false);  
      bridgeLevel.addValueChangeListener (new ValueChangeListener() {
         public void valueChange (ValueChangeEvent e) {
            Boolean val = (Boolean)(e.getValue());
            if (val.booleanValue()) {        
               ctrBridgeMesh =  cMesh.copy (); 
               Vector3d vec0 = new Vector3d(op.normal);
               vec0.scale (opDist-10); 
               ctrBridgeMesh.translate (vec0);
               fixedCtrBridge = new FixedMeshBody("fixedCtrBridge", ctrBridgeMesh);
               mech.addMeshBody (fixedCtrBridge);
               RenderProps.setAlpha (fixedCtrBridge, 0.4);
               RenderProps.setFaceStyle (fixedCtrBridge,FaceStyle.FRONT_AND_BACK);
               RenderProps.setFaceColor (fixedCtrBridge,new Color(190, 231, 255)); //blue
               RenderProps.setVisible (fixedCtrBridge, false);
              
               intersectorBridge = new Intersector (target, ctrBridgeMesh, 0, Color.cyan); //10
               RenderProps.setFaceColor (intersectorBridge, violet); 
               mech.addRenderable (intersectorBridge);

               currentHeight =10;
               mech.removeMeshBody (fixedCtrSlider);
               ctrSliderMesh= ctrBridgeMesh.copy (); 
               fixedCtrSlider = new FixedMeshBody("fixedCtrSlider", ctrSliderMesh);
               mech.addMeshBody (fixedCtrSlider);
               RenderProps.setAlpha (fixedCtrSlider, 0.2);
               RenderProps.setFaceStyle (fixedCtrSlider,FaceStyle.FRONT_AND_BACK);
               RenderProps.setFaceColor (fixedCtrSlider, Color.green); 
               distanceLabel.setText("Vertical distance to the occlusal plane = 10.0 mm"); 
               heightSlider.setValue (100);
               RenderProps.setVisible (fixedCtrSlider, false);
               
               dentureLevel.getLabel ().setEnabled (false);
               dentureLevel.getCheckBox ().setEnabled (false); 
               dentureLevel.setValue (false);               
               renderRange.getLabel ().setEnabled (false);
               renderRange.getCheckBox ().setEnabled (false);
               
               }
            else {
               mech.removeRenderable (intersectorBridge);
               mech.removeMeshBody (fixedCtrBridge);
               dentureLevel.getLabel ().setEnabled (true);
               dentureLevel.getCheckBox ().setEnabled (true); 
               if(fixedCtrDenture!=null && fixedCtrBridge!=null) {
                  renderRange.getLabel ().setEnabled (true);
                  renderRange.getCheckBox ().setEnabled (true);
                  }
               }
            }
         });
      
      dentureLevel = new BooleanSelector("Implant denture (15 mm)", false);
      donorHeightPanel.addExtraWidget (dentureLevel);
      dentureLevel.getLabel ().setEnabled (false);
      dentureLevel.getCheckBox ().setEnabled (false); 
      dentureLevel.addValueChangeListener (new ValueChangeListener() {
         public void valueChange (ValueChangeEvent e) {
            Boolean val = (Boolean)(e.getValue());
            if (val.booleanValue()) {
               ctrDentureMesh =  cMesh.copy ();
               Vector3d vec0 = new Vector3d(op.normal);
               vec0.scale (opDist-15);
               ctrDentureMesh.translate (vec0);
               fixedCtrDenture = new FixedMeshBody("fixedCtrDenture", ctrDentureMesh);
               mech.addMeshBody (fixedCtrDenture);
               RenderProps.setAlpha (fixedCtrDenture, 0.4);
               RenderProps.setFaceStyle (fixedCtrDenture,FaceStyle.FRONT_AND_BACK);
               RenderProps.setFaceColor (fixedCtrDenture,new Color(131, 209, 255)); //blue
              
               intersectorDenture = new Intersector (target, ctrDentureMesh, 0, Color.cyan);
               RenderProps.setFaceColor (intersectorDenture, new Color (1f, 1f, 0.8f));   //bone          
               mech.addRenderable (intersectorDenture);
               RenderProps.setVisible (fixedCtrDenture, false);
               
               currentHeight =15;
               mech.removeMeshBody (fixedCtrSlider);
               ctrSliderMesh= ctrDentureMesh.copy (); 
               fixedCtrSlider = new FixedMeshBody("fixedCtrSlider", ctrSliderMesh);
               mech.addMeshBody (fixedCtrSlider);
               RenderProps.setAlpha (fixedCtrSlider, 0.2);
               RenderProps.setFaceStyle (fixedCtrSlider,FaceStyle.FRONT_AND_BACK);
               RenderProps.setFaceColor (fixedCtrSlider, Color.green); 
               distanceLabel.setText("Vertical distance to the occlusal plane = 15.0 mm"); 
               heightSlider.setValue (150);
               RenderProps.setVisible (fixedCtrSlider, false);
               
               bridgeLevel.getLabel ().setEnabled (false);
               bridgeLevel.getCheckBox ().setEnabled (false); 
               bridgeLevel.setValue (false);                       
               renderRange.getLabel ().setEnabled (false);
               renderRange.getCheckBox ().setEnabled (false);
               
               }
            else {
               mech.removeRenderable (intersectorDenture);
               mech.removeMeshBody (fixedCtrDenture);
               bridgeLevel.getLabel ().setEnabled (true);
               bridgeLevel.getCheckBox ().setEnabled (true); 
               if(fixedCtrDenture!=null && fixedCtrBridge!=null) {
                  renderRange.getLabel ().setEnabled (true);
                  renderRange.getCheckBox ().setEnabled (true);
                  }
               }
            }
         });
      
      renderRange = new BooleanSelector("Render the cut range", false);
      donorHeightPanel.addExtraWidget (renderRange);
      renderRange.getLabel ().setEnabled (false);
      renderRange.getCheckBox ().setEnabled (false);
      renderRange.addValueChangeListener (new ValueChangeListener() {
         public void valueChange (ValueChangeEvent e) {
            Boolean val = (Boolean)(e.getValue());
            if(fixedCtrDenture!=null && fixedCtrBridge!=null) {
               if (val.booleanValue()) {             
               
                  mech.removeRenderable (intersectorBridge);
                  mech.removeRenderable (intersectorBridge);
                  mech.addRenderable (intersectorDenture);
                  mech.addRenderable (intersectorBridge);
                  intersectorBridge.setContoursOnly (false);
                  intersectorBridge.setRenderDiffMesh (true);
               
                  intersectorDenture.setContoursOnly (false);
                  intersectorDenture.setRenderDiffMesh (true);
                  RenderProps.setVisible (intersectorDenture, true);
                  RenderProps.setVisible (intersectorBridge, true);
                  
                  RenderProps.setVisible (fixedCtrDenture, false);
                  RenderProps.setVisible (fixedCtrBridge, false);               
                  RenderProps.setVisible (fixedCtrSlider, false);
               }           
               else {
              
                  RenderProps.setVisible (intersectorBridge, false);
                  RenderProps.setVisible (intersectorDenture, false);               
                  intersectorBridge.setRenderDiffMesh (false);                 
                  intersectorDenture.setRenderDiffMesh (false);
                  dentureLevel.setValue (false);
                  bridgeLevel.setValue (false);
                  }
               }
            }
         });

      showSliderMesh = new BooleanSelector("The slider mesh", false);
      donorHeightPanel.addExtraWidget (showSliderMesh);
      showSliderMesh.getLabel ().setEnabled (false);
      showSliderMesh.getCheckBox ().setEnabled (false);
      showSliderMesh.addValueChangeListener (new ValueChangeListener() {
         public void valueChange (ValueChangeEvent e) {
            Boolean val = (Boolean)(e.getValue());
            if (val.booleanValue()) {             
               RenderProps.setVisible (fixedCtrSlider, true);
            }
            else {
               RenderProps.setVisible (fixedCtrSlider, false);
            }
         }
      });
      
      heightSlider = new JSlider(50, 350, 100);
      heightSlider.setPaintTrack(true); 
      heightSlider.setPaintTicks(true); 
      heightSlider.setPaintLabels(true); 
      heightSlider.setMajorTickSpacing(50); 
      heightSlider.setMinorTickSpacing(10); 
      Hashtable<Integer,JLabel> scalelabelTable =
      new Hashtable<Integer,JLabel> ();
      scalelabelTable.put (new Integer (100), new JLabel ("10 mm"));
      scalelabelTable.put (new Integer (200), new JLabel ("20 mm"));
      scalelabelTable.put (new Integer (300), new JLabel ("30 mm"));
        
      distanceLabel.setEnabled (false);
      distanceLabel.setText("Vertical distance to the occlusal plane = " 
      + heightSlider.getValue()/10.0 +" mm"); 
      
      heightSlider.setLabelTable (scalelabelTable);
      heightSlider.addMouseListener (new MouseAdapter () {
         @Override
         public void mouseReleased (MouseEvent e) {           
            showSliderMesh.setValue (true);       
            RenderProps.setVisible (fixedCtrSlider, true);
            double sliderValue = heightSlider.getValue()/10.0;
            distanceLabel.setText("Vertical distance to the occlusal plane =" 
            + sliderValue +" mm");
            Vector3d vec = new Vector3d(op.normal);
            vec.scale (currentHeight-sliderValue);
            ctrSliderMesh.translate (vec);
            currentHeight =sliderValue;
            
            if(heightSlider.getValue()== 100) {
               bridgeLevel.setValue (true);
            }
            else if(heightSlider.getValue()== 150) {               
               dentureLevel.setValue (true);             
            }            
            rerender();
         }
      });    
      donorHeightPanel.addExtraWidget (heightSlider);
      heightSlider.setEnabled (false); 
      donorHeightPanel.addExtraWidget (distanceLabel);      
      
      panel.addWidget (donorHeightPanel);     
      addControlPanel(panel);             
   }
   
   public double getControlRadius() {
      return manualEditRadius;
   }
 
   public void startRegistration(){
      ICPRegistration myRegister = new ICPRegistration();
      AffineTransform3d X = new AffineTransform3d();
      myRegister.registerICP (X, target, sourcePolyMesh, 3, 6);
      fem.transformGeometry(X);

      myIntersector = new Intersector (target, cMesh, 0, Color.red);
      myContours = myIntersector.getContours ();
      mech.addRenderable (myIntersector);
     // veryStartTime = System.nanoTime(); 
      
      DynamicRegistrationController controller = new DynamicRegistrationController (mech);      
      GMMMeshCorrespondence gmm = new GMMMeshCorrespondence ();
      gmm.setNoiseFraction (0.01); //omega
      gmm.setNearestK (10);  // consider 10 nearest points
      
      controller.addRegistrationTarget (source, target, 1.0, gmm);
      controller.setForceScaling (fScaling);    //beta    
      controller.setName("registration");
      addController(controller);
      addController (new TimeCheck());
        
      // remove the correspondence web between source and target
      RenderProps.setVisible (controller.getTargetMeshes (), false);  
      RenderProps.setFaceColor (myIntersector, new Color (1f, 1f, 0.8f));
   }
 
   // Rui: move the center of mesh to origin, align
   public static Point3d transformMeshOrigin(PolygonalMesh mesh)
   {
           ArrayList<Vertex3d> verts = mesh.getVertices();
           Point3d com = new Point3d();
           
           //Advanced For Loop
           for (Vertex3d vertex3d : verts) 
                com.add(vertex3d.getPosition());
           
           com.scale(1.0/verts.size());
           com.negate();
           mesh.translate(com);          
           return com;
   }
    
   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);      
      GLViewer viewer = driver.getViewer ();
      if (viewer != null) {
         viewer.setBackgroundColor (Color.WHITE);
         viewer.setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      }        
      
      viewer.addMouseWheelListener(new MouseWheelListener(){
         public void mouseWheelMoved(MouseWheelEvent e) {
             if (selectionNum==1) {            
                Main.getMain ().setMouseWheelZoomScale (0);
                int notches = e.getWheelRotation();
                RenderProps.setVisible (sphereBody, true);
                
                radiusScaling = Math.pow (1.03, notches);   
                double currR = manualEditRadius;
                manualEditRadius *= radiusScaling;
                if(manualEditRadius>20) {
                   radiusScaling= 20/currR;
                   manualEditRadius = 20;
                }
                if(manualEditRadius<3) {
                   radiusScaling= 3/currR;
                   manualEditRadius = 3;
                }
                sphere.scale (radiusScaling); 
                Point3d tr = new Point3d();
                tr.set (currPos);
                tr.scaledAdd (-radiusScaling, currPos);
                sphere.translate (tr);               
                rerender();
             }
             else {
                Main.getMain ().setMouseWheelZoomScale (10);
                e.getComponent().getParent().dispatchEvent(e);
                } 
             }
         });
      
      viewer.addMouseInputListener (new MouseInputListener () {
         public void mouseReleased (MouseEvent e) {
            
            if(SwingUtilities.isLeftMouseButton(e)){
//               System.out.println(" selectionNum " + selectionNum);
//               System.out.println(" controlPoints.size () " + controlPoints.size ());
               if(selectionNum==controlPoints.size ()&& selectionNum !=0 ) {
                  myContours= myIntersector.getContours ();
                  ctrlPntsInsertion();
                  renderCtrlPnts();
                  
                  Main.getMain ().getSelectionManager ().clearSelections ();
                  for(Point p:controlPoints) 
                     Main.getMain ().getSelectionManager ().addSelected (p);
                       
                  if(selectionNum==controlPoints.size ()&& selectionNum !=0 ) {
                     selectedPnt = controlPoints.get (0);
                     selectedIdx = 0;
                     Point3d prePos = new Point3d(currPos);
                     currPos = new Point3d(selectedPnt.getPosition ()); 
                     Point3d tr = new Point3d();
                     tr.sub (currPos, prePos);
                     sphere.translate (tr);         
                     selectionNum = controlPoints.size ();
                     }
                  }
               }         
             }

         @Override
         public void mouseClicked (MouseEvent e) {}
         @Override
         public void mousePressed (MouseEvent e) {}
         @Override
         public void mouseEntered (MouseEvent e) {}
         @Override
         public void mouseExited (MouseEvent e) {}
         @Override
         public void mouseDragged (MouseEvent e) {}
         @Override         
         public void mouseMoved (MouseEvent e) {}
   
      });
   }
   
   protected FemModel3d createVoxelizedFem(
      FemModel3d fem, 
      PolygonalMesh mesh, 
      int res) {
      fem = EmbeddedFem.createVoxelizedFem(fem, mesh, RigidTransform3d.IDENTITY, res, 0);
      
      // since mesh is not actually closed, can cause some issues with inside/outside tests, remove stray elements
      ArrayList<FemElement3d> toremove = new ArrayList<> ();
      BVFeatureQuery bvq = new BVFeatureQuery ();
      Point3d centroid = new Point3d();
      Point3d near = new Point3d();
      for (FemElement3d elem : fem.getElements ()) {
         elem.computeCentroid (centroid);
         bvq.nearestFaceToPoint (near, null, mesh, centroid);
         if (near.distance (centroid) > 10) { //0.01  5
            toremove.add (elem);
         }
      }
      
      for (FemElement3d elem : toremove) {
         fem.removeElement (elem);
      }
      
      ArrayList<FemNode3d> rnodes = new ArrayList<>();
      for (FemNode3d node : fem.getNodes ()) {
         if (node.numAdjacentElements () < 1) {
            rnodes.add (node);
         }
      }
      for (FemNode3d node : rnodes) {
         fem.removeNode (node);
      }
          
      return fem;
   }

   // run registration for 2 units of simulation time   
   private class TimeCheck extends ControllerBase {     
      public TimeCheck () {}     
      public void apply (double t0, double t1) {
         if(t1 == 0.01)
            veryStartTime = System.nanoTime(); 
         if(t1 < runTime) {
            instructions.setText(modelName); 
            instructions.append("\n\nConducting registration...");
            instructions.setForeground (Color.gray);   
            }
         if(t1 == runTime) {                              
            contourProcessing();
            computeOP();
            
            }           
      }
   }
     
   private void contourProcessing() {
      long startTime = System.nanoTime(); 
//      StopWatch timer = new StopWatch("timer");
//      timer.checkpoint("Start editing");
      
      BufferedWriter out = null;
      try {
          FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
             "timer/regtime.txt"), true); 
          out = new BufferedWriter(fstream);     
          long estimatedTime = startTime-veryStartTime;
          double t= ((double)estimatedTime)/1000000000.0;
          DecimalFormat numberFormat = new DecimalFormat("#.00");
          out.write(numberFormat.format(t));
          out.close ();
      }
      catch (IOException e) {
          System.err.println("Error: " + e.getMessage());
      }  
      
      fem.removeMeshComp (cMeshComp); // get the fixedcMesh deformed      
      mech.remove (cMeshComp);

      instructions.setText(modelName); 
      instructions.append ("\n\nSelecting control points... ");            
      instructions.setForeground (Color.gray);  
      
      ArrayList<ArrayList<Vertex3d>> keyVerts = new ArrayList<ArrayList<Vertex3d>>();
      //System.out.println("myContours0 "+ myContours.size ());
      myContours = myIntersector.getContours ();
      CSGContourProcess.projection (myContours, keyVerts, target, cMesh, 45);
      
//      int numKey =0;
     // target.setVertexColoringEnabled ();
//      for(int i=0; i<keyVerts.size (); i++) {              
//         for(int j=0; j<keyVerts.get (i).size (); j++) {
//            
//            Point point = new Point(keyVerts.get (i).get (j).pnt);
//            mech.addPoint (point);
//            RenderProps.setPointColor(point, Color.pink);
//            RenderProps.setPointRadius(point, 0.7);
//            RenderProps.setPointStyle(point, PointStyle.SPHERE);
//            RenderProps.setVisible(point, true); 
//            RenderProps.setShading (point, Shading.SMOOTH);
//            numKey++;
//            }
//         }
//      System.out.println("numKey "+ numKey);
      
      instructions.setText(modelName); 
      instructions.append ("\n\nChecking SDF value...");  
      instructions.append ("\nRunning snake algorithm...");
      instructions.setForeground (Color.gray);  
      progress.setStringPainted(true);
      for(int i=0; i<keyVerts.size (); i++) {   
         System.out.println("Contour "+ i +" has " + keyVerts.get (i).size ()+" control points");
         System.out.println("   Checking sdf... "); 
         keyVerts.set (i, CSGContourProcess.checkSDF (keyVerts.get (i), target));  
        
//         for(int j=0; j<keyVerts.get (i).size (); j++) {
//            Point point = new Point(keyVerts.get (i).get (j).pnt);
//            mech.addPoint (point);
//            RenderProps.setPointColor(point, new Color(178,255,255)); //blue
//            RenderProps.setPointRadius(point, 0.7);
//            RenderProps.setPointStyle(point, PointStyle.SPHERE);
//            RenderProps.setVisible(point, true); 
//            RenderProps.setShading (point, Shading.SMOOTH);
//         }
         System.out.println("   Running snake... ");
         int snakeIter =3;
         if(target.numVertices ()<5000)
            snakeIter =2;
         else if(target.numVertices ()>50000)
            snakeIter =5;
         for(int t=0; t<snakeIter; t++) {  // 3 for 10k-20k-vertex meshes
            progress.setValue (t+1);
            keyVerts.set (i, CSGContourProcess.snakeCurv(keyVerts.get (i), target)); 
         }
 
//         for(int j=0; j<keyVerts.get (i).size (); j++) {
//            Point point = new Point(keyVerts.get (i).get (j).pnt);
//            mech.addPoint (point);
//            RenderProps.setPointColor(point, new Color(60,217,59)); //green
//            RenderProps.setPointRadius(point, 0.7);
//            RenderProps.setPointStyle(point, PointStyle.SPHERE);
//            RenderProps.setVisible(point, true); 
//            RenderProps.setShading (point, Shading.SMOOTH);
//         }
      }
   
      System.out.println("Snake completed! ");
      progress.setValue (3);    
//      PolygonalMesh sMesh = cMesh.copy (); 
//      FixedMeshBody fixedsMesh = new FixedMeshBody("fixeds", sMesh);
//      mech.addMeshBody (fixedsMesh);
//      RenderProps.setVisible (fixedsMesh, true);
//      RenderProps.setAlpha (fixedsMesh, 0.2); //0.2
//      RenderProps.setDrawEdges (fixedsMesh, true);
//      RenderProps.setEdgeColor (fixedsMesh, Color.cyan);
//      Intersector myIntersector2 = new Intersector (target, sMesh, 0, Color.cyan);
//      mech.addRenderable (myIntersector2);
      
      int numKey =0;
       for(int i=0; i<keyVerts.size (); i++) {              
          for(int j=0; j<keyVerts.get (i).size (); j++) {
             numKey++;
             }
          }
      Point3d[] ccPoints = new Point3d[numKey];
      int n=0;
      for(int i=0; i<keyVerts.size (); i++) {   
         for(int j=0; j<keyVerts.get (i).size (); j++) {
            ccPoints[n] = new Point3d(keyVerts.get (i).get (j).getPosition ());
            n++;
         }
         }
      
     // TPSSurfaceFit.thinPlateSpline (cMesh,ccPoints );
      CSGContourProcess.gaussianDeform(keyVerts, cMesh); 
      cMesh.notifyVertexPositionsModified();
      rerender();
      myContours=myIntersector.getContours (); //error for 025
      
      instructions.setText(modelName); 
      instructions.append ("\n\nSnake completed! ");
      instructions.setForeground (bleu);  
      isCtrlPntsSet =true;                    
      renderDiffMesh.getLabel ().setEnabled (true);
      renderDiffMesh.getCheckBox ().setEnabled (true);
      
      initialContour.getLabel ().setEnabled (true);
      initialContour.getCheckBox ().setEnabled (true); 
      bridgeLevel.getLabel ().setEnabled (true);
      bridgeLevel.getCheckBox ().setEnabled (true); 
      dentureLevel.getLabel ().setEnabled (true);
      dentureLevel.getCheckBox ().setEnabled (true); 
      opVisible.getLabel ().setEnabled (true);
      opVisible.getCheckBox ().setEnabled (true); 
      renderRange.getLabel ().setEnabled (false);
      renderRange.getCheckBox ().setEnabled (false);
      showSliderMesh.getLabel ().setEnabled (true);
      showSliderMesh.getCheckBox ().setEnabled (true);
      heightSlider.setEnabled (true);
  
      //timer.checkpoint("End editing");
           
      try {
          FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
             "timer/snaketime.txt"), true); 
          out = new BufferedWriter(fstream);     
          long estimatedTime = System.nanoTime() - startTime;
          double t= ((double)estimatedTime)/1000000000.0;
          DecimalFormat numberFormat = new DecimalFormat("#.00");
          out.write(numberFormat.format(t));
          out.close ();
      }
      catch (IOException e) {
          System.err.println("Error: " + e.getMessage());
      }  
      //RenderProps.setVisible (fixedCMesh, true);  //true
      startOfManualTime = System.nanoTime();
      
   }
    
   private void ctrlPntsInsertion() {     
      ctrlPnts3d.clear ();
      for(int i=0; i<myContours.size (); i++) {             
         ArrayList<Point3d> keyTemp = new ArrayList<Point3d>();  
         int interval = myContours.get (i).size ()/20;
         for(int j =0; j<interval+1; j++) {
            IntersectionPoint intersectPnt = new IntersectionPoint();
            if (myContours.get (i).size ()==20* j)
               intersectPnt = myContours.get (i).get (20* j-1);
            else
               intersectPnt = myContours.get (i).get (20* j);
            
            if(j>0) {
               double dist = intersectPnt.distance (keyTemp.get (keyTemp.size ()-1));              
               if(dist>9) 
                  keyTemp.add(myContours.get (i).get (20*j-10));
               }
            keyTemp.add (intersectPnt);
            }
         
         if(keyTemp.size ()>2) {
            double dist2 =keyTemp.get (0).distance (keyTemp.get (keyTemp.size ()-1));
            if(dist2>9) 
               keyTemp.add(myContours.get (i).get (myContours.get (i).size ()-8));
            }
         ctrlPnts3d.add (keyTemp);   
         }      
   }
          
   private void renderCtrlPnts() { 
      if(controlPoints !=null) {
         mech.clearPoints ();
      }
      
      originalPos.clear();
      controlPoints.clear (); 
      for(int i=0; i<ctrlPnts3d.size (); i++) {
         for (int j=0; j< ctrlPnts3d.get (i).size (); j++){  
            Point point = new Point(ctrlPnts3d.get (i).get (j));
            mech.addPoint (point);
            controlPoints.add(point); 
            Point3d pos = new Point3d(point.getPosition ());
            originalPos.add (pos);          
         }        
      }
      
      RenderProps.setSphericalPoints (mech, 0.7, violet);
      RenderProps.setShading (mech.points (), Shading.SMOOTH);
      startOfManualTime2=System.nanoTime();
   }
     
   private void computeOP() {

      int numV = opvMesh.numVertices ();
      MatrixNd A = new MatrixNd (3, numV);
      Point3d centerPnt = new Point3d (0,0,0);
      for(int i =0; i< numV; i++) {
         Vertex3d v = opvMesh.getVertex (i);  
         centerPnt.add (v.pnt);
      }      
      centerPnt.x /= numV;
      centerPnt.y /= numV;
      centerPnt.z /= numV;
      
      for(int i =0; i< numV; i++) {
         Vertex3d v = opvMesh.getVertex (i);
         Vector3d s = new Vector3d();
         A.setColumn (i, s.sub (v.pnt, centerPnt));
      }
     
      SVDecomposition svd = new SVDecomposition();
      svd.factor (A);
      MatrixNd U = svd.getU ();         
      Vector3d pNormal = new Vector3d();
      U.getColumn (2, pNormal);    
      //System.out.println(" pNormal " + pNormal);
      if(pNormal.z<0)
         pNormal.negate ();
      op = new Plane(pNormal, centerPnt); 
      
      baseOP = RigidBody.createBox ("base", 100, 100, 0.1, 0.2);
      RigidTransform3d T = new RigidTransform3d (centerPnt.x, centerPnt.y, centerPnt.z);
      RotationMatrix3d R = new RotationMatrix3d();
      R = ShapeDiameterFunction.computeRotation(new Vector3d (0, 0, -1), pNormal);
      T.mulRotation (R);      
      baseOP.setPose (T);         
      baseOP.setDynamic (false);
      mech.addRigidBody (baseOP);
      RenderProps.setAlpha(baseOP, 0.2);  
      RenderProps.setFaceColor (baseOP, Color.green); //green
      RenderProps.setVisible (baseOP, false);
   
      int numCV =0;
      for(int i=0; i<cMesh.numVertices (); i++) {
         Vertex3d v = cMesh.getVertex (i);
         if(op.distance (v.pnt) <-5) {
            numCV++;
            opDist += op.distance (v.pnt);
            //System.out.println("op.distance (v.pnt) " + op.distance (v.pnt));
         }         
      }
      opDist /= -numCV;
      currentHeight=opDist;
       
      int initialV = (int)(10*opDist);
      heightSlider.setValue (initialV);
      distanceLabel.setText("Vertical distance to the occlusal plane = " + 
      initialV/10.0 +" mm");
      
      ctrSliderMesh= cMesh.copy (); 
      fixedCtrSlider = new FixedMeshBody("fixedCtrSlider", ctrSliderMesh);
      mech.addMeshBody (fixedCtrSlider);
      RenderProps.setVisible (fixedCtrSlider, false);
      RenderProps.setAlpha (fixedCtrSlider, 0.2);
      RenderProps.setFaceStyle (fixedCtrSlider,FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor (fixedCtrSlider, Color.green); 
      
   }
    
   public void prerender (RenderList rlist) {        
      if(myIntersector!= null
      && myIntersector.getContours ().isEmpty ()==true) {
         do {
            cMesh.translate (new Point3d(0,0,-2));
         }while(myIntersector.getContours ().isEmpty ()==true);
      }
      else if(selectedPnt != null 
      && selectedPnt.distance (currPos)>0.05 
      && selectedPnt.distance (currPos)<5) {             
         if(selectedPnt.distance (originalPos.get (selectedIdx))<20) {        
            Point3d tr = new Point3d();
            tr.sub(selectedPnt.getPosition (), currPos);
            sphere.translate (tr); 
            currPos.set (selectedPnt.getPosition ());

            if(selectionNum==1) {
               CSGContourProcess.gaussianManualDeformSingleP(
                  tr, selectedPnt, vertDistTemp, controlPoints, target, manualEditRadius, 
                  oneSidedControl.getBooleanValue (), clipToSurface.getBooleanValue ());            
            }
            else if(selectionNum==controlPoints.size ()) { 
                  cMesh.translate (new Point3d(0,0,tr.z));
                  BVFeatureQuery query = new BVFeatureQuery();  
                  for(int i=0; i<controlPoints.size (); i++) {         
                     Point p =controlPoints.get (i);
                     if(target.pointIsInside (p.getPosition ())==1) {  //1 if <code>pnt</code> is inside
                        Point3d nearPnt = new Point3d();
                        query.nearestFaceToPoint (nearPnt, null, target, p.getPosition ());
                        p.setPosition (nearPnt);
                     }
                  }
            }            
            else {
               //x y z             
               CSGContourProcess.gaussianManualDeformMultiP(tr, mcPList, vertDistTempList,
                  target,clipToSurface.getBooleanValue ());    
               }
            }
         cMesh.notifyVertexPositionsModified();
      }      
         
      super.prerender(rlist);
   }
   
   public void ctrlPointsSelection() {
      oneSidedControl.getLabel ().setEnabled (true);
      oneSidedControl.getCheckBox ().setEnabled (true);
      clipToSurface.getLabel ().setEnabled (true);
      clipToSurface.getCheckBox ().setEnabled (true);
      instructions.setText(modelName); 
      instructions.append ("\n\nControl point selection.");
      instructions.setForeground (bleu);
    
      SelectionManager mySelectionManager = Main.getMain ().getSelectionManager ();          
      mySelectionManager.addSelectionListener (new SelectionListener() {
         public void selectionChanged(SelectionEvent e) {
            ModelComponent mc = e.getLastAddedComponent();              
            selectionNum = mySelectionManager.getNumSelected ();
            
            if(mc==null && selectionNum ==0) {
               RenderProps.setVisible (sphereBody, false);  
               oneSidedControl.getLabel ().setEnabled (false);
               oneSidedControl.getCheckBox ().setEnabled (false);
               oneSidedControl.setValue (false);
               clipToSurface.getLabel ().setEnabled (false);
               clipToSurface.getCheckBox ().setEnabled (false);
               clipToSurface.setValue (false);
               selectedPnt= null;
               selectionNum=0;
               instructions.setText(modelName); 
               instructions.append ("\n\nControl point selection.");              
               instructions.setForeground (bleu);
            }
            else {  
             // if (mc instanceof Point) {
                 LinkedList<ModelComponent> mcList= mySelectionManager.getCurrentSelection ();
                 Point prePnt = selectedPnt;
                 if(mc instanceof Point) {
                    selectedPnt = (Point)mc;
                    selectedIdx = mc.getNumber ();
                 }
                 else {
                    selectedPnt = (Point)mcList.get (0);
                    selectedIdx = mcList.get (0).getNumber ();
                    //selectedIdx = controlPoints.indexOf ((Point)mc);                  
                 }
           
               Point3d prePos = new Point3d(currPos);
               currPos = new Point3d(selectedPnt.getPosition ()); 
               Point3d tr = new Point3d();
               tr.sub (currPos, prePos);
               sphere.translate (tr);   
              
               if(selectionNum==1) {
                  RenderProps.setVisible (sphereBody, true); 
                  oneSidedControl.getLabel ().setEnabled (true);
                  oneSidedControl.getCheckBox ().setEnabled (true);
                  oneSidedControl.setValue (false);
                  clipToSurface.getLabel ().setEnabled (true);
                  clipToSurface.getCheckBox ().setEnabled (true);
                  clipToSurface.setValue (false);
                 
                  if(prePnt != selectedPnt) {
                     vertDistTemp.clear ();                    
                     for(int i=0; i<cMesh.numVertices ();i++) {                 
                        double dist = cMesh.getVertex (i).distance (((Point)mc).getPosition ());
//                        double dist=0;
//                        if(cMesh.getVertex (i).pnt.z-((Point)mc).getPosition ().z<0)
//                           dist = Math.sqrt (Math.pow (cMesh.getVertex (i).pnt.x-((Point)mc).getPosition ().x,2)
//                           +Math.pow (cMesh.getVertex (i).pnt.y-((Point)mc).getPosition ().y,2));
//                        else
//                           dist = cMesh.getVertex (i).distance (((Point)mc).getPosition ());
                        if(dist<20 ) {
                           GeoVertex vd = new GeoVertex(cMesh.getVertex (i),dist);
                           vertDistTemp.add(vd);           
                           }
                        }                 
                     MinCostComparator costComparator = new MinCostComparator();
                     Collections.sort(vertDistTemp, costComparator);                                                           
  
                     }
                  instructions.setText(modelName); 
                  instructions.append ("\n\n1 point selected.");
                  instructions.setForeground (bleu); 
                  }
               else if(selectionNum==controlPoints.size ()) {
                  RenderProps.setVisible (sphereBody, false);
                  oneSidedControl.getLabel ().setEnabled (false);
                  oneSidedControl.setValue (false);
                  oneSidedControl.getCheckBox ().setEnabled (false);
                  clipToSurface.getLabel ().setEnabled (false);
                  clipToSurface.getCheckBox ().setEnabled (false);
                  clipToSurface.setValue (true);
                                   
                  instructions.setText(modelName); 
                  instructions.append ("\n\nAll points selected. ");
                  instructions.setForeground (bleu);
               }
               else {                                    
                  RenderProps.setVisible (sphereBody, false);
                  oneSidedControl.getLabel ().setEnabled (false);
                  oneSidedControl.getCheckBox ().setEnabled (false);
                  oneSidedControl.setValue (false);
                  clipToSurface.getLabel ().setEnabled (true);
                  clipToSurface.getCheckBox ().setEnabled (true);
                  clipToSurface.setValue (false);
                                  
                  mcPList.clear ();
                  vertDistTempList.clear ();
                  for(int t=0;t<mcList.size ();t++) {
                     ModelComponent mcP= mcList.get (t);
                     if (mcP instanceof Point) {
                        mcPList.add (((Point)mcP));
                        vertDistTemp= new ArrayList<GeoVertex>();
                 
                        for(int i=0; i<cMesh.numVertices ();i++) {   
                           Vertex3d v = cMesh.getVertex (i);
                           double dist = v.distance (((Point)mcP).getPosition ());
                           if(dist<12) {
                              if(t==0){
                                 GeoVertex vd = new GeoVertex(cMesh.getVertex (i),dist);
                                 vertDistTemp.add(vd);           
                                 }
                              else {
                                 boolean isAdded= false;
                                 outerloop:
                                 for(int j=0;j<t;j++) {
                                    for(int k=0;k<vertDistTempList.get (j).size ();k++) {
                                       if(vertDistTempList.get (j).get (k).getVert ()==v) {
                                          isAdded=true;
                                          if(vertDistTempList.get (j).get (k).getCost ()>dist) {
                                             vertDistTempList.get (j).remove (k);                                          
                                             GeoVertex vd = new GeoVertex(cMesh.getVertex (i),dist);
                                             vertDistTemp.add(vd);
                                             break outerloop;
                                          }
                                       }                             
                                    }
                                 }
                                 if(isAdded==false) {
                                    GeoVertex vd = new GeoVertex(cMesh.getVertex (i),dist);
                                    vertDistTemp.add(vd);   
                                    }                      
                                 }
                              }
                        }             
                        MinCostComparator costComparator = new MinCostComparator();
                        Collections.sort(vertDistTemp, costComparator);               
                        vertDistTempList.add (vertDistTemp);
                     }
                  }  
                  
                  for(int i=0; i<vertDistTempList.size (); i++) {
                     if(vertDistTempList.get (i).size ()==0) {
                        vertDistTempList.remove (i);
                        mcPList.remove (i);
                        i--;
                     }
                  }                                
                  instructions.setText(modelName); 
                  instructions.append ("\n\n"+ selectionNum+ " points selected. ");
                  instructions.setForeground (bleu); 
                  }
               
               rerender();
               }              
            }
         });         
   }
   
   public void rebuild() {
      removeModel (mech);
      mech = null;
      fem.clear ();
      ctrlPnts3d.clear ();
      controlPoints.clear ();
      originalPos.clear ();
      vertDistTemp.clear ();
      vertDistTempList.clear ();
      mcPList.clear ();
      removeAllControllers ();
      clearRenderables ();    
      Main.getMain ().getSelectionManager ().clearSelections ();
      Main.getMain ().getMainFrame ().getSelectCompPanelHandler().clear();
      
      ClassFilter myClassFilter = new ClassFilter (Point.class);
      Main.getMain ().getSelectionManager ().addFilter (myClassFilter);
      Main.getMain ().getSelectionManager ().filterSelections (myClassFilter);
      
      mech = new MechModel("mech");
      mech.setGravity (0, 0, 0);
      addModel(mech);
      
      String meshDirectory = ArtisynthPath.getSrcRelativePath(this, "mandible/");
      sourceMeshName = "mandible_3kv.stl";       
      String cMeshName = "16clip.stl"; // contour mesh template  
      String opvName = "OP.stl"; //occlusal plane vertices
      runTime = 1;
      fDensity =0.0001; //0.0001
      fScaling = 1e6;  //1e6
         
   // load source mesh (template with teeth)           
      sourcePolyMesh = GenericModel.loadGeometry(meshDirectory, sourceMeshName);
      Point3d tr = new Point3d();
      tr = transformMeshOrigin(sourcePolyMesh); 
      
   // create a FEM for source mesh (template with teeth)
      fem = createVoxelizedFem(null, sourcePolyMesh, 15);
      fem.setName("fem");
      fem.setDensity(fDensity); 
      source = fem.addMesh("source", sourcePolyMesh);  
      fem.setMaterial(new LinearMaterial(100, 0.3));    
      mech.addModel(fem);
        
    // load contour mesh and put it in the FEM
      cMesh = GenericModel.loadGeometry(meshDirectory, cMeshName); 
      cMesh.translate (tr);      
      cMeshComp = fem.addMesh("cMesh", cMesh);  
      fixedCMesh = new FixedMeshBody("fixedCMesh", cMesh);
      mech.addMeshBody (fixedCMesh);
      
      opvMesh = GenericModel.loadGeometry(meshDirectory, opvName); 
      opvMesh.translate (tr);      
      opvComp = fem.addMesh("opv", opvMesh);  

      manualEditRadius = 10;
      radiusScaling =1;      
      sphere = MeshFactory.createSphere(manualEditRadius,48);
      sphereBody = new FixedMeshBody("Sphere",sphere);
      mech.addMeshBody (sphereBody);     
      
      RenderProps.setVisible (fem.getNodes (), false);
      RenderProps.setVisible (fem.getElements (), false);
      RenderProps.setVisible (source, false);     
      RenderProps.setVisible (cMeshComp, false); 
      RenderProps.setAlpha (fixedCMesh, 0.2); //0.2
      RenderProps.setDrawEdges (fixedCMesh, true);
      RenderProps.setEdgeColor (fixedCMesh, Color.cyan);
//      RenderProps.setAlpha (fixedCMesh, 0.2); //0.2
//      RenderProps.setFaceStyle (fixedCMesh,FaceStyle.FRONT_AND_BACK);
//      RenderProps.setFaceColor (fixedCMesh, Color.cyan);
      RenderProps.setVisible (opvComp, false); 
      RenderProps.setVisible (sphereBody, false);
      RenderProps.setAlpha (sphereBody, 0.2);
      RenderProps.setFaceColor (sphereBody, violet);
      RenderProps.setShading (sphereBody, Shading.SMOOTH);
      //RenderProps.setFaceColor (source, Color.pink);
          
      fixedCtrDenture=null;
      fixedCtrBridge=null;
      progress.setStringPainted(false);
      ctrlPntReassignment.setEnabled (false);
      allPntSelection.setEnabled (false);      
      renderDiffMesh.getLabel ().setEnabled (false);
      renderDiffMesh.setValue (false);
      renderDiffMesh.getCheckBox ().setEnabled (false);
      
      smoothing.getLabel ().setEnabled (false);
      smoothing.setValue (false);
      smoothing.getCheckBox ().setEnabled (false);
      
      oneSidedControl.setValue (false);
      oneSidedControl.getLabel ().setEnabled (false);
      oneSidedControl.getCheckBox ().setEnabled (false); 
      clipToSurface.setValue (false);
      clipToSurface.getLabel ().setEnabled (false);
      clipToSurface.getCheckBox ().setEnabled (false); 
      smoothedMesh=null;
      myIntersector=null;
      myContours.clear ();      
      selectedPnt = null;
      selectedIdx=-1;
      selectionNum=0;
      currPos= new Point3d();
      isCtrlPntsSet = false; //false
     
      opDist=0;
      initialContour.setValue (true);
      initialContour.getLabel ().setEnabled (false);
      initialContour.getCheckBox ().setEnabled (false); 
      bridgeLevel.setValue (false);
      bridgeLevel.getLabel ().setEnabled (false);
      bridgeLevel.getCheckBox ().setEnabled (false); 
      dentureLevel.setValue (false);
      dentureLevel.getLabel ().setEnabled (false);
      dentureLevel.getCheckBox ().setEnabled (false); 
      opVisible.setValue (false);
      opVisible.getLabel ().setEnabled (false);
      opVisible.getCheckBox ().setEnabled (false); 
      renderRange.setValue (false);
      renderRange.getLabel ().setEnabled (false);
      renderRange.getCheckBox ().setEnabled (false); 
      heightSlider.setValue (100);
      heightSlider.setEnabled (false);
      showSliderMesh.getLabel ().setEnabled (false);
      showSliderMesh.getCheckBox ().setEnabled (false);
//      RenderProps.setVisible (fixedCtrSlider, false);
//      RenderProps.setVisible (fixedCtrBridge, false);
//      RenderProps.setVisible (fixedCtrDenture, false);
      
   }
   
   public boolean getMenuItems(List<Object> items) {
      
      JMenuItem loadingItem = GuiUtils.createMenuItem (
         this, "Load a mandible", "");
      items.add (loadingItem);
      
      JMenuItem settingsItem = GuiUtils.createMenuItem (
         this, "Edit the contour", "Manually adjust the contour");
      items.add (settingsItem);
      if(isCtrlPntsSet)
         settingsItem.setEnabled (true);
      else
         settingsItem.setEnabled (false);
       
      JMenuItem item = GuiUtils.createMenuItem (
         this, "Save the edentulous model", "");

         if (smoothedMesh != null) { 
         item.setEnabled (true);
      }
      else {
        item.setEnabled (false);         
      }
      items.add (item);
      
      return true;
   }   

   public void actionPerformed (ActionEvent event) {  
      if (event.getActionCommand().equals("Load a mandible")) {
        
         Main main = Main.getMain();
         JFrame frame = main.getMainFrame();
         String initialFileDir = ArtisynthPath.getSrcRelativePath(this, "mandible/");
         JFileChooser chooser = new JFileChooser(initialFileDir);
         ExtensionFileFilter filter = new ExtensionFileFilter ("OBJ, STL, PLY files", "obj", "stl", "ply");
         chooser.setFileFilter (filter);
         int retVal = chooser.showOpenDialog (frame);  
         String fileDir;
         if (retVal == JFileChooser.APPROVE_OPTION) {
            fileDir=chooser.getCurrentDirectory().getPath ();             
            File file = chooser.getSelectedFile();                        
            Main.getMain ().reset ();  
            if (fixedTarget!= null) 
               rebuild();

            modelName = file.getName ();
            target = GenericModel.loadGeometry(fileDir+ "/", file.getName ()); 
            System.out.println("Loading mesh " + file.getName () + "..."); 

            BufferedWriter out = null;
            try {
               FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
                  "timer/regtime.txt"), true); 
               out = new BufferedWriter(fstream);
               out.write("\n"+ file.getName () +" ");
               out.close ();
            }
            catch (IOException e) {
               System.err.println("Error: " + e.getMessage());
            }

            try {
               FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
                  "timer/snaketime.txt"), true); 
               out = new BufferedWriter(fstream);
               out.write("\n"+ file.getName ()+" ");
               out.close ();
            }
            catch (IOException e) {
               System.err.println("Error: " + e.getMessage());
            }

            try {
               FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
                  "timer/manualtime.txt"), true); 
               out = new BufferedWriter(fstream);
               out.write("\n"+ file.getName () +" ");
               out.close ();
            }
            catch (IOException e) {
               System.err.println("Error: " + e.getMessage());
            }
            
            if (target.numVertices () >50000) {         
               fDensity =0.00001;
               fScaling = 1000;
            }

            targetTr = transformMeshOrigin(target);
            fixedTarget = new FixedMeshBody("Target", target);
            mech.addMeshBody(fixedTarget);
            RenderProps.setAlpha (fixedTarget, 1);
            RenderProps.setFaceColor (fixedTarget, new Color (1f, 1f, 0.8f));               
            startRegistration();
            Main.getMain ().play (runTime); 

            instructions.setText(modelName); 
            instructions.append("\n\nReady to start mesh registration. "); 
            instructions.append ("\n(Click 'Start simulation' button.) ");
            instructions.setForeground (bleu);

         }
      }
          
      else if (event.getActionCommand().equals("Save the edentulous model")) {
         if (smoothedMesh != null) {  
            targetTr.negate ();
            smoothedMesh.translate(targetTr); 
            BufferedWriter out = null;
            try {
               FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
                  "timer/manualtime.txt"), true); 
               out = new BufferedWriter(fstream);     
               long estimatedTime = System.nanoTime()-startOfManualTime;
               long estimatedTime2 = System.nanoTime()-startOfManualTime2;
               double t= ((double)estimatedTime)/1000000000.0;
               double t2= ((double)estimatedTime2)/1000000000.0;
               DecimalFormat numberFormat = new DecimalFormat("#.00");
               out.write(numberFormat.format(t));
               out.write(" "+ numberFormat.format(t2));
               out.close ();
               
               EditorUtils.saveMesh (smoothedMesh, null); 
               targetTr.negate ();
               smoothedMesh.translate(targetTr);  
               instructions.setText(modelName);
               instructions.setForeground (bleu);  
           }
           catch (IOException e) {
               System.err.println("Error: " + e.getMessage());              
           }
         }
      }
      
      else if (event.getActionCommand().equals ("Edit the contour")) {
         myContours = myIntersector.getContours ();   
         ctrlPntsInsertion();
         renderCtrlPnts();
         ctrlPointsSelection();   
         isCtrlPntsSet = false;
         oneSidedControl.getLabel ().setEnabled (false);
         oneSidedControl.getCheckBox ().setEnabled (false);
         clipToSurface.getLabel ().setEnabled (false);
         clipToSurface.getCheckBox ().setEnabled (false);
         ctrlPntReassignment.setEnabled (true);
         allPntSelection.setEnabled (true);  
         mech.add (cMeshComp);
         
      }  
      
      else if (event.getSource () == ctrlPntReassignment) {
         myContours = myIntersector.getContours (); 
         ctrlPntsInsertion();
         renderCtrlPnts();
      }
      
      else if (event.getSource () == allPntSelection) {
         if(selectionNum != controlPoints.size ()) {
            for(Point p:controlPoints) 
               Main.getMain ().getSelectionManager ().addSelected (p);
                              
            selectedPnt = controlPoints.get (0);
            selectedIdx = 0;
            Point3d prePos = new Point3d(currPos);
            currPos = new Point3d(selectedPnt.getPosition ()); 
            Point3d tr = new Point3d();
            tr.sub (currPos, prePos);
            sphere.translate (tr);
   
            selectionNum = controlPoints.size ();
            RenderProps.setVisible (sphereBody, false);
            oneSidedControl.getLabel ().setEnabled (false);
            oneSidedControl.setValue (false);
            oneSidedControl.getCheckBox ().setEnabled (false);
            clipToSurface.getLabel ().setEnabled (false);
            clipToSurface.getCheckBox ().setEnabled (false);
            clipToSurface.setValue (true);
                         
            instructions.setText(modelName); 
            instructions.append ("\n\nAll points selected. ");
            instructions.setForeground (bleu);
         }
      }
   } 

}
