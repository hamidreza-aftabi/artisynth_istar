 package artisynth.istar.Rui;
//Rui: from artisynth_models_registration
import java.awt.Color;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
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
import java.util.LinkedList;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.swing.SwingUtilities;
import javax.swing.event.MouseInputListener;

import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.editorManager.EditorUtils;
import artisynth.core.gui.selectionManager.ClassFilter;
import artisynth.core.gui.selectionManager.SelectionEvent;
import artisynth.core.gui.selectionManager.SelectionListener;
import artisynth.core.gui.selectionManager.SelectionManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ExtensionFileFilter;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.collision.IntersectionContour;
import maspack.collision.IntersectionPoint;
import maspack.geometry.MeshFactory;
import maspack.geometry.MeshUtilities;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.properties.PropertyList;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.Renderer.Shading;
import maspack.widgets.BooleanSelector;
import maspack.widgets.GuiUtils;
import maspack.widgets.ValueChangeEvent;
import maspack.widgets.ValueChangeListener;

public class CSGSegManual extends RootModel {
   
   PolygonalMesh cMesh;  //contour mesh, clipmesh
   PolygonalMesh mandible;
   FixedMeshBody fixedMandible;
   FixedMeshBody fixedCMesh;
   FixedMeshBody fixedSmoothedMandible;
   Point3d targetTr = new Point3d();
   PolygonalMesh smoothedMandible;  
   static ArrayList<ArrayList<Point3d>> ctrlPnts3d = new ArrayList<ArrayList<Point3d>>();
   double manualEditRadius = 7;
   double radiusScaling =1;
   PolygonalMesh sphere = MeshFactory.createSphere(manualEditRadius,48);
   FixedMeshBody sphereBody = new FixedMeshBody();   
   Point selectedPnt = null;
   Point3d currPos= new Point3d();
   JTextArea instructions;
   BooleanSelector renderDiffMesh;
   BooleanSelector smoothing;
   BooleanSelector oneSidedControl;
   BooleanSelector clipToSurface;   
   JButton ctrlPntReassignment;
   JButton allPntSelection;
   Intersector myIntersector; 
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
   private static Color vert = new Color(0.18f, 0.64f, 0.31f);   
   long startOfManualTime=System.nanoTime();  
   int selectionNum=0;
   ArrayList<Point> mcPList = new ArrayList<Point>();
   String modelName;
   
   public static PropertyList myProps =
   new PropertyList (CSGSegManual.class, RootModel.class);
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
     
      sphereBody = new FixedMeshBody("Sphere",sphere);
      mech.addMeshBody (sphereBody);
      RenderProps.setVisible (sphereBody, false);
      RenderProps.setAlpha (sphereBody, 0.2);
      RenderProps.setFaceColor (sphereBody, violet);
      RenderProps.setShading (sphereBody, Shading.SMOOTH);
     
      // Only objects of point class are selectable
      ClassFilter myClassFilter = new ClassFilter (Point.class);     
      Main.getMain ().getSelectionManager ().addFilter (myClassFilter);
      Main.getMain ().getSelectionManager ().filterSelections (myClassFilter);
      
      Main.getMain ().setTimelineVisible (false);
      panel = new ControlPanel("Manual editing"); 
      panel.addLabel ("Messages"); 
      
      instructions = new JTextArea ("Ready to load a new model. ",  3, 1); 
      instructions.setForeground (bleu);
      instructions.setEditable (false);
      panel.addWidget (instructions); 
      
      renderDiffMesh = new BooleanSelector("Remove teeth", false);
      panel.addWidget (renderDiffMesh);
      renderDiffMesh.getLabel ().setEnabled (false);
      renderDiffMesh.getCheckBox ().setEnabled (false);
      renderDiffMesh.addValueChangeListener (new ValueChangeListener() {
         public void valueChange (ValueChangeEvent e) {
            Boolean val = (Boolean)(e.getValue());
            if (val.booleanValue()) {                
               if(myIntersector != null) {
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
                           instructions.setForeground (Color.red); 
                           break;
                        }
                     }
                     if(ctrIsOpen ==false) {
                        RenderProps.setVisible (fixedMandible, false);                    
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
                        instructions.append ("\n\nTeeth removed!");
                        instructions.setForeground (bleu);
                        }                  
                     }
                  }
               } 
            else { // teeth back
               RenderProps.setVisible (fixedMandible, true);
               myIntersector.setContoursOnly (true);
               myIntersector.setRenderDiffMesh (false); 
               smoothing.getLabel ().setEnabled (false);
               smoothing.getCheckBox ().setEnabled (false);
               oneSidedControl.getLabel ().setEnabled (true);
               oneSidedControl.getCheckBox ().setEnabled (true);
               clipToSurface.getLabel ().setEnabled (true);
               clipToSurface.getCheckBox ().setEnabled (true);
               
               if(controlPoints.size ()!=0){
                  for(Point p : controlPoints)
                     RenderProps.setVisible (p, true);
                  RenderProps.setVisible (fixedCMesh, true); 
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
                  smoothedMandible =  myIntersector.getDiffMesh ().copy ();                 
                  MeshUtilities.closeHoles (smoothedMandible, 0);                
                  //smoothedMandible.mergeCloseVertices (0.1);
                  for(int i=0; i< myIntersector.getContours ().size (); i++) {
                     CSGContourProcess.aveSmoothing(
                        myIntersector.getContours ().get (i), smoothedMandible);  
                     }                  
                  }
               instructions.setText(modelName); 
               instructions.append ("\n\nSmoothing completed!");              
               instructions.setForeground (bleu); 
               
               RenderProps.setVisible (myIntersector, false); 
               fixedSmoothedMandible = new FixedMeshBody("fixedSmoothed", smoothedMandible);
               mech.add (fixedSmoothedMandible);
               RenderProps.setAlpha (fixedSmoothedMandible, 1);
               RenderProps.setFaceColor (fixedSmoothedMandible, new Color (1f, 1f, 0.8f));     
               
               renderDiffMesh.getLabel ().setEnabled (false);
               renderDiffMesh.getCheckBox ().setEnabled (false);
               } 
            else {
               RenderProps.setVisible (myIntersector, true);
               smoothedMandible=null;
               RenderProps.setVisible (fixedSmoothedMandible, false);
               mech.remove (fixedSmoothedMandible);
               renderDiffMesh.getLabel ().setEnabled (true);
               renderDiffMesh.getCheckBox ().setEnabled (true);
               
               instructions.setText(modelName);               
               instructions.setForeground (bleu);                              
            }
            
            rerender();             
            }
         });
      
      panel.addLabel ("Point selection settings");  
//      panel.addWidget ("Point radius", mech.points(),"renderProps.pointRadius",0.5,2);
//      panel.addWidget (sphereBody, "Sphere radius",sphere.getRadius (),20);
//      myProps.add (
//         "sphereRadius", "The sphere radius to use.", sphere.getRadius ());
         
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
      
      clipToSurface = new BooleanSelector("Clip on to the surface", false);
      panel.addWidget (clipToSurface);
      clipToSurface.getLabel ().setEnabled (false);
      clipToSurface.getCheckBox ().setEnabled (false); 
      
      oneSidedControl = new BooleanSelector("One-sided control", false);
      panel.addWidget (oneSidedControl);
      oneSidedControl.getLabel ().setEnabled (false);
      oneSidedControl.getCheckBox ().setEnabled (false);   
        
      addControlPanel(panel);      
       
   }
   
   public double getControlRadius() {
      return manualEditRadius;
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
         @Override
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
               if(selectionNum==controlPoints.size () && selectionNum !=0 ) {
                  myContours= myIntersector.getContours ();
                  ctrlPntsInsertion();
                  renderCtrlPnts();
                  
                  Main.getMain ().getSelectionManager ().clearSelections ();
                  for(Point p:controlPoints) 
                     Main.getMain ().getSelectionManager ().addSelected (p);
                                    
                  if(selectionNum==controlPoints.size () && selectionNum !=0 ) {
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
   }
     
   public void prerender (RenderList rlist) {      
//      System.out.println("selectedPnt.getPosition ().z " + selectedPnt.getPosition ().z);
//      System.out.println("currPos.z"+ currPos.z);      
      if(myIntersector!= null) {
         if( myIntersector.getContours ().isEmpty ()==true) {
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
              
               if(selectionNum==controlPoints.size ()) {
                  cMesh.translate (new Point3d(0,0,tr.z));
                  }
               else if(selectionNum==1) {
                  // System.out.println("vertDistTemp " + vertDistTemp.get (1).getVert ().pnt);
                 // System.out.println("tr"+ tr); 
                  CSGContourProcess.gaussianManualDeformSingleP(
                     tr, selectedPnt, vertDistTemp, controlPoints, mandible, manualEditRadius, 
                     oneSidedControl.getBooleanValue (), clipToSurface.getBooleanValue ()); 
               }       
               else {
                  //x y z                                          
                  CSGContourProcess.gaussianManualDeformMultiP(tr, mcPList, vertDistTempList,
                     mandible,clipToSurface.getBooleanValue ());   
                  }
               }
         }
        cMesh.notifyVertexPositionsModified();
      }               
      super.prerender(rlist);
   }
   
   public void ctrlPntsSelection() {
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
                  if(selectedPnt ==null)
                     System.out.println("selectedPnt ======= null ");
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
                        if(dist<20) {
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
                  clipToSurface.setValue (false);
                                   
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
                        // System.out.println("cal dist ");
                        for(int i=0; i<cMesh.numVertices ();i++) {   
                           Vertex3d v = cMesh.getVertex (i);
                           double dist = v.distance (((Point)mcP).getPosition ());
                           if(dist<7) {
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
      addModel(mech);
      
      manualEditRadius = 7;
      radiusScaling =1;      
      sphere = MeshFactory.createSphere(manualEditRadius,48);
      sphereBody = new FixedMeshBody("Sphere",sphere);
      mech.addMeshBody (sphereBody);            
      RenderProps.setVisible (sphereBody, false);
      RenderProps.setAlpha (sphereBody, 0.2);
      RenderProps.setFaceColor (sphereBody, violet);
      RenderProps.setShading (sphereBody, Shading.SMOOTH);
     
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
      smoothedMandible=null;
      myIntersector=null;
      myContours.clear ();      
      selectedPnt = null;
      selectedIdx=-1;
      selectionNum=0;
      currPos= new Point3d();
     
   }
   
   public boolean getMenuItems(List<Object> items) {
      
      JMenuItem loadingItem = GuiUtils.createMenuItem (
         this, "Load a mandible", "");
      items.add (loadingItem);
      
      JMenuItem item = GuiUtils.createMenuItem (
         this, "Save the edentulous model", "");

//    if (myIntersector !=null && myIntersector.getDiffMesh ()!=null 
//    && renderDiffMesh.getBooleanValue ()==true) {       
//       item.setEnabled (true);
//    }
      if (smoothedMandible != null) { 
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
         String mandibleFileDir = ArtisynthPath.getSrcRelativePath(this, "mandible/proposedT48/");
         String cMeshFileDir = ArtisynthPath.getSrcRelativePath(this, "mandible/savedCmesh/");
         JFileChooser chooser = new JFileChooser(mandibleFileDir);
         ExtensionFileFilter filter = new ExtensionFileFilter ("OBJ, STL or PLY files", "obj", "stl", "ply");
         chooser.setFileFilter (filter);
         int retVal = chooser.showOpenDialog (frame);
         if (retVal == JFileChooser.APPROVE_OPTION) {
            File file = chooser.getSelectedFile();                       
           // Main.getMain ().getScheduler ().reset ();                
            if (myIntersector!= null) 
               rebuild();

            Main.getMain().getViewer().setEye(new Point3d(0, -250, 0));            
            modelName = file.getName ();
            mandible = GenericModel.loadGeometry(mandibleFileDir, file.getName ()); 
            targetTr = transformMeshOrigin(mandible);
            fixedMandible = new FixedMeshBody("Mandible", mandible);
            mech.addMeshBody(fixedMandible);
            
            RenderProps.setAlpha (fixedMandible, 1);
            RenderProps.setFaceColor (fixedMandible, new Color (1f, 1f, 0.8f)); 
           
            System.out.println("Loading mesh " + file.getName () + "...");            
            cMesh = GenericModel.loadGeometry(cMeshFileDir, 
               file.getName ().substring(0, file.getName ().length() - 4)+"_CMesh.stl"); 
            
            fixedCMesh = new FixedMeshBody("CMesh", cMesh);
            mech.addMeshBody(fixedCMesh);
            RenderProps.setAlpha (fixedCMesh, 0.2); //0.2
//            RenderProps.setFaceStyle (fixedCMesh,FaceStyle.FRONT_AND_BACK);
//            RenderProps.setFaceColor (fixedCMesh, Color.cyan);
       
            RenderProps.setDrawEdges (fixedCMesh, true);
            RenderProps.setEdgeColor (fixedCMesh, Color.cyan);
            
            myIntersector = new Intersector (mandible, cMesh, 0, Color.red);
            myContours = myIntersector.getContours ();
            mech.addRenderable (myIntersector);
            RenderProps.setFaceColor (myIntersector, new Color (1f, 1f, 0.8f));
            
            ctrlPntsInsertion();
            renderCtrlPnts();
            ctrlPntsSelection();          
            oneSidedControl.getLabel ().setEnabled (false);
            oneSidedControl.getCheckBox ().setEnabled (false);
            clipToSurface.getLabel ().setEnabled (false);
            clipToSurface.getCheckBox ().setEnabled (false);
            ctrlPntReassignment.setEnabled (true);
            allPntSelection.setEnabled (true); 
            renderDiffMesh.getLabel ().setEnabled (true);
            renderDiffMesh.getCheckBox ().setEnabled (true);
            
            instructions.setText(modelName); 
            instructions.append("\n\nReady for tooth removal/manual editing ");
            instructions.setForeground (bleu);
            startOfManualTime = System.nanoTime();
                       
            BufferedWriter out = null;
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
         }
      }
          
      else if (event.getActionCommand().equals("Save the edentulous model")) {
         if (smoothedMandible != null) {  
            targetTr.negate ();
            smoothedMandible.translate(targetTr); 
            BufferedWriter out = null;
            try {
               FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
                  "timer/manualtime.txt"), true); 
               out = new BufferedWriter(fstream);     
               long estimatedTime = System.nanoTime()-startOfManualTime;             
               double t= ((double)estimatedTime)/1000000000.0;               
               DecimalFormat numberFormat = new DecimalFormat("#.00");
               out.write(numberFormat.format(t));              
               out.close ();
               
               EditorUtils.saveMesh (smoothedMandible, null); 
               targetTr.negate ();
               smoothedMandible.translate(targetTr);  
               instructions.setText(modelName); 
               instructions.setForeground (bleu);  
           }
           catch (IOException e) {
               System.err.println("Error: " + e.getMessage());              
           }
         }   
         
//         else if(myIntersector.getDiffMesh ()!=null) {                      
//            targetTr.negate ();
//            myIntersector.getDiffMesh ().translate(targetTr);      
//            EditorUtils.saveMesh (myIntersector.getDiffMesh (), null);  
//            targetTr.negate ();
//            myIntersector.getDiffMesh ().translate(targetTr); 
//                     
//         }
      }
      
     
      else if (event.getSource () == ctrlPntReassignment) {
         myContours = myIntersector.getContours (); 
         ctrlPntsInsertion();
         renderCtrlPnts();
         RenderProps.setVisible (sphereBody, false);
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
            clipToSurface.setValue (false);
            
              
            instructions.setText(modelName); 
            instructions.append ("\n\nAll points selected. ");
            instructions.setForeground (bleu);
         }
      }
   } 

}
