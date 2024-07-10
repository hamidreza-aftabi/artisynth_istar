 package artisynth.istar.Rui;
//Rui: from artisynth_models_registration
import java.awt.Color;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JTextArea;
import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.gui.selectionManager.SelectionEvent;
import artisynth.core.gui.selectionManager.SelectionListener;
import artisynth.core.gui.selectionManager.SelectionManager;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.geometry.ICPRegistration;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.Vector3d;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.Renderer.PointStyle;
import maspack.widgets.GuiUtils;
import maspack.widgets.StringField;

public class SVDRemoveT extends RootModel {
   
   PolygonalMesh cMesh;
   PolygonalMesh sourcePolyMesh;
   PolygonalMesh target;
   FixedMeshBody fixedTarget;
   FixedMeshBody fixedCMesh;
   ArrayList<Vertex3d> ctrVerts = new  ArrayList<Vertex3d>();
   ArrayList<Vertex3d> keyVerts = new  ArrayList<Vertex3d>(); 
   int iterSnake =2;
   int keyPntsIdx0;  //smaller
   int keyPntsIdx1;
   boolean confirmed = true;
   ArrayList<Vertex3d> selectedKeyVerts = new  ArrayList<Vertex3d>();
   ArrayList<GeoVertex> geoPath;
   ArrayList<Vertex3d> vPath  = new  ArrayList<Vertex3d>();
   //JButton feaReroute;
   JButton geoReroute;
   JButton substitute;
   JButton reject;
   JButton pntsRemoval;
  // JButton undoTR;
   Boolean mouseClicked = false;
   JTextArea instructions;
   boolean isTeethRemovable = false;
   boolean isControlPntsDisplayable = false;
   double runTime = 0.5;
   double fDensity = 0.0001;
   StringField rTime;
   StringField fDen;
   JProgressBar progress;   
   ArrayList<Point> controlPnts = new  ArrayList<Point>();
   MechModel mech = new MechModel("mech");
   
   // Rui: @ is Java Annotations, means that the method is overriding the parent class. 
   // Rui: The super keyword is used to access and call functions on an object's parent.
   @Override
   public void build(String[] args) throws IOException {
      super.build(args);
      
      //MechModel mech = new MechModel("mech");
      addModel(mech);
      
      // turn off gravity for registration
      mech.setGravity(0, 0, 0);
           
      String meshDirectory = ArtisynthPath.getSrcRelativePath(this, "mandible/");
      String targetMeshName = "proposedT48/003-Mandible.stl";   // 
      String sourceMeshName = "mandible_3kv.stl";    
      String cMeshName = "16clip.stl"; // contour mesh
     
      // load source mesh (template with teeth)
      System.out.println("Loading mesh " + sourceMeshName + "...");
      sourcePolyMesh = GenericModel.loadGeometry(meshDirectory, sourceMeshName);
      transformMeshOrigin(sourcePolyMesh);
        
      // load target mesh
      System.out.println("Loading mesh " + targetMeshName + "...");               
      target = GenericModel.loadGeometry(meshDirectory, targetMeshName);
      transformMeshOrigin(target);
      fixedTarget = new FixedMeshBody("target", target);
      mech.addMeshBody(fixedTarget);
      
      ICPRegistration myRegister = new ICPRegistration();
      AffineTransform3d X = new AffineTransform3d();
      myRegister.registerICP (X, target, sourcePolyMesh, 3, 12);
      sourcePolyMesh.transform (X);
      
      // load contour mesh
      System.out.println("Loading mesh " + cMeshName + "...");
      cMesh = GenericModel.loadGeometry(meshDirectory, cMeshName);  
      fixedCMesh = new FixedMeshBody("fixedCMesh", cMesh);
      //fixedCMesh.transformMesh (X);
      mech.addMeshBody(fixedCMesh);
             
      RenderProps.setAlpha(fixedTarget, 1);    
      RenderProps.setAlpha(fixedCMesh, 0.9);
      RenderProps.setFaceColor(fixedCMesh, Color.MAGENTA);      
          
      int verticesNum = cMesh.numVertices ();      
      ArrayList<GeoVertex> geoVertices = new ArrayList<GeoVertex>(); 
      MinCostComparator costComparator = new MinCostComparator();
      Vector3d baseV = new Vector3d(-1, 0,0);
      double minY = 100;
      double minX = 100;
      double maxX = -100;
      
      for(int i =0; i< verticesNum; i++) {
         Vertex3d v = cMesh.getVertex (i);
         if(v.pnt.y> 0)
            geoVertices.add (new GeoVertex(v, baseV.angle (new Vector3d(v.pnt.x, v.pnt.y,0))));   
         if(v.pnt.y<= 0 && v.pnt.x>0)
            geoVertices.add (new GeoVertex(v, 6.28-baseV.angle (new Vector3d(v.pnt.x, v.pnt.y,0))));
         
         if(v.pnt.y<= 0 && v.pnt.x<0)
            geoVertices.add (new GeoVertex(v, -baseV.angle (new Vector3d(v.pnt.x, v.pnt.y,0))));
         
         if(v.pnt.y <minY)
            minY = v.pnt.y;
         if(v.pnt.x <minX)
            minX = v.pnt.x;
         if(v.pnt.x >maxX)
            maxX = v.pnt.x;
      }      
      Collections.sort(geoVertices, costComparator); // ascending
      
      ArrayList <Integer> idxList = new ArrayList <Integer> ();
      int numP =7;
      for(int i =0; i<numP+1; i++)
         idxList.add (i*geoVertices.size ()/numP);
      
      ArrayList<Vertex3d> plCtr = new ArrayList<Vertex3d>();
      double aveEdgeLength = target.computeAverageEdgeLength ();
      
      for(int i = 0; i<numP; i++) {
      
         Point3d centerPnt = new Point3d (0,0,0);
         int vSize = idxList.get (i+1)- idxList.get (i);        
         double minAng = geoVertices.get (idxList.get (i)).getCost ();
         double maxAng = geoVertices.get (idxList.get (i+1)-1).getCost ();
         
         for(int j =idxList.get (i); j< idxList.get (i+1); j++) {
            Vertex3d v = geoVertices.get (j).getVert ();  
            centerPnt.x += v.pnt.x;
            centerPnt.y += v.pnt.y;
            centerPnt.z += v.pnt.z;
         }
         
//         System.out.println("maxX " + i + "..." + maxX);
//         System.out.println("minX " + i + "..." + minX);
//         System.out.println("maxY " + i + "..." + maxY);
//         System.out.println("minY " + i + "..." + minY);
         
         centerPnt.x /= vSize;
         centerPnt.y /= vSize;
         centerPnt.z /= vSize;
         MatrixNd A = new MatrixNd (3, vSize);
         
         for(int j =idxList.get (i); j< idxList.get (i+1); j++) {
            Vertex3d v = geoVertices.get (j).getVert ();
            Vector3d s = new Vector3d();
            A.setColumn (j-idxList.get (i), s.sub (v.pnt, centerPnt));
         }
        
         SVDecomposition svd = new SVDecomposition();
         svd.factor (A);
         MatrixNd U = svd.getU ();         
         Vector3d pNormal = new Vector3d();
         U.getColumn (2, pNormal);         
         Plane p = new Plane(pNormal, centerPnt); 
                           
         for(Vertex3d vert: target.getVertices ()) {
            if(vert.pnt.y > minY  && vert.pnt.x > minX  && vert.pnt.x < maxX) {
               double vertAng = baseV.angle (new Vector3d(vert.pnt.x, vert.pnt.y,0));           
               if(vertAng <maxAng && vertAng > minAng) {
                  double distToP = p.distance (vert.pnt);
                  if(Math.abs (distToP)< aveEdgeLength/3) 
                     plCtr.add (vert);
                  }         
               }
            }
//       RigidBody base = RigidBody.createBox ("base" +i, 20, 20, 0.1, 0.2);
//       RigidTransform3d T = new RigidTransform3d (centerPnt.x, centerPnt.y, centerPnt.z);
//       RotationMatrix3d R = new RotationMatrix3d();
//       R = ShapeDiameterFunction.computeRotation(new Vector3d (0, 0, -1), pNormal);
//       T.mulRotation (R);      
//       base.setPose (T);         
//       base.setDynamic (false);
//       mech.addRigidBody (base);
//       RenderProps.setAlpha(base, 1);  
//       RenderProps.setFaceColor (base, Color.cyan);
      }
 //     target.setVertexColoringEnabled();      
//      for(int k = 0; k<plCtr.size (); k++) {
//         target.setColor(plCtr.get(k).getIndex (),Color.cyan);         
 //     }
      
   }
 
   // Rui: move the center of mesh to origin, align
   public static Point3d transformMeshOrigin(PolygonalMesh mesh)
   {
           ArrayList<Vertex3d> verts = mesh.getVertices();
           Point3d com = new Point3d();
           
           //Advanced For Loop
           for (Vertex3d vertex3d : verts) {
                com.add(vertex3d.getPosition());
           }
           com.scale(1.0/verts.size());
           com.negate();
           mesh.translate(com);
           
           return com;
   }
   
   @Override
   public void attach(DriverInterface driver) {
      super.attach(driver);
      
      Main.getMain ().setTimelineVisible (false);
      ControlPanel panel = new ControlPanel("controls");
       
//      JPanel rVars = new JPanel ();
//      rVars.setLayout (new GridLayout (0, 2));
//      rVars.add(rTime);
//      rVars.add(fDen);
//      panel.addWidget (rVars);
      
      
      panel.addLabel ("Messages"); 
      instructions = new JTextArea ("Ready to start registration. ",  3, 1); 
      instructions.append ("\n(click 'start simulation' button) ");
      instructions.setForeground (Color.gray);
      instructions.setEditable (false);
      panel.addWidget (instructions); 
      
      progress = new JProgressBar();
      progress.setMinimum(0);
      progress.setMaximum(iterSnake+1);
      panel.addWidget (progress);
           
      geoReroute = new JButton("Shortest path") ;
      geoReroute.addActionListener (this);
      geoReroute.addMouseListener(new java.awt.event.MouseAdapter() {
         String lastInstruction;
         Color lastColor;
         public void mouseEntered(java.awt.event.MouseEvent evt) {
            lastInstruction = instructions.getText ();
            lastColor = instructions.getForeground ();
            instructions.setText ("Find the shortest path between 2 selected keypoints.");
            instructions.append ("\nSuitable for rerouting for short distance");
            instructions.setForeground (Color.gray);
            }
         public void mouseExited(java.awt.event.MouseEvent evt) {
            if(mouseClicked==false) {
             instructions.setText (lastInstruction);
             instructions.setForeground (lastColor);
            }
            if(mouseClicked=true)
               mouseClicked=false;
         }
     });
      
//      feaReroute = new JButton("Featured path") ;
//      feaReroute.addActionListener (this);
//      feaReroute.addMouseListener(new java.awt.event.MouseAdapter() {
//         String lastInstruction;
//         Color lastColor;
//         public void mouseEntered(java.awt.event.MouseEvent evt) {
//            lastInstruction = instructions.getText ();
//            lastColor = instructions.getForeground ();
//            instructions.setText ("Find a path attracted to features between 2 selected keypoints.");
//            instructions.append ("\nSuitable for rerouting for long distance");
//            instructions.setForeground (Color.gray);
//            }
//         public void mouseExited(java.awt.event.MouseEvent evt) {
//            if(mouseClicked==false) {
//             instructions.setText (lastInstruction);
//             instructions.setForeground (lastColor);
//            }
//            if(mouseClicked=true)
//               mouseClicked=false;
//         }
//     });
      
      substitute = new JButton("Accept") ;
      substitute.addActionListener (this);
      substitute.addMouseListener(new java.awt.event.MouseAdapter() {
         String lastInstruction;
         Color lastColor;
         public void mouseEntered(java.awt.event.MouseEvent evt) {
            lastInstruction = instructions.getText ();
            lastColor = instructions.getForeground ();
            instructions.setText ("Substitute the path between the two selected keypoints with the new path.");
            instructions.setForeground (Color.gray);
            }
         public void mouseExited(java.awt.event.MouseEvent evt) {
            if(mouseClicked==false) {
             instructions.setText (lastInstruction);
             instructions.setForeground (lastColor);
            }
            if(mouseClicked=true)
               mouseClicked=false;
         }
     });
      
      reject = new JButton("Reject") ;
      reject.addActionListener (this);
      reject.addMouseListener(new java.awt.event.MouseAdapter() {
         String lastInstruction;
         Color lastColor;
         public void mouseEntered(java.awt.event.MouseEvent evt) {
            lastInstruction = instructions.getText ();
            lastColor = instructions.getForeground ();
            instructions.setText ("Reject the new path.");
            instructions.setForeground (Color.gray);
            }
         public void mouseExited(java.awt.event.MouseEvent evt) {
            if(mouseClicked==false) {
             instructions.setText (lastInstruction);
             instructions.setForeground (lastColor);
            }
            if(mouseClicked=true)
               mouseClicked=false;
         }
     });
      
      
      pntsRemoval = new JButton("Remove a point") ;
      pntsRemoval.addActionListener (this);
      pntsRemoval.addMouseListener(new java.awt.event.MouseAdapter() {
         String lastInstruction;
         Color lastColor;
         public void mouseEntered(java.awt.event.MouseEvent evt) {
            lastInstruction = instructions.getText ();
            lastColor = instructions.getForeground ();
            instructions.setText ("Remove one selected keypoint from the contour.");
            instructions.setForeground (Color.gray);
            }
         public void mouseExited(java.awt.event.MouseEvent evt) {
            if(mouseClicked==false) {
             instructions.setText (lastInstruction);
             instructions.setForeground (lastColor);
            }
            if(mouseClicked=true)
               mouseClicked=false;
            
         }
     });
      
//      undoTR = new JButton("Teeth back") ;
//      undoTR.addActionListener (this);
//      undoTR.addMouseListener(new java.awt.event.MouseAdapter() {
//         String lastInstruction;
//         Color lastColor;
//         public void mouseEntered(java.awt.event.MouseEvent evt) {
//            lastInstruction = instructions.getText ();
//            lastColor = instructions.getForeground ();
//            instructions.setText ("Undo teeth removal");
//            instructions.setForeground (Color.gray);
//            }
//         public void mouseExited(java.awt.event.MouseEvent evt) {
//            if(mouseClicked==false) {
//             instructions.setText (lastInstruction);
//             instructions.setForeground (lastColor);
//            }
//            if(mouseClicked=true)
//               mouseClicked=false;
//            
//         }
//     });
//      
      
      JPanel manualFix = new JPanel ();
      manualFix.setLayout (new GridLayout (0, 2));
      //feaReroute.setEnabled (false);
      geoReroute.setEnabled (false);
      substitute.setEnabled (false);
      reject.setEnabled (false);
      pntsRemoval.setEnabled (false);
      //undoTR.setEnabled (false);
      manualFix.add(pntsRemoval);
      manualFix.add(geoReroute);
      //manualFix.add(feaReroute);
      manualFix.add(substitute);  
      manualFix.add(reject); 
      //manualFix.add (undoTR);
      panel.addWidget (manualFix);
            
      addControlPanel(panel);
      
      GLViewer viewer = driver.getViewer ();
      if (viewer != null) {
         viewer.setBackgroundColor (Color.WHITE);
         viewer.setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      }
      
   }
   
   public void renderKeyVerts(ArrayList<Vertex3d> vertices, Color color ) {
      
      controlPnts.clear ();
      double size = 0.5;
      for (Vertex3d vert: vertices){  
          Point point = new Point(vert.pnt);
          controlPnts.add(point);
          mech.add (point);
          RenderProps.setPointColor(point, color);
          RenderProps.setPointRadius(point, size);
          RenderProps.setPointStyle(point, PointStyle.SPHERE);
          RenderProps.setVisible(point, true);                  
      }
      
   } 


   public void removeTeeth() {
      boolean removeT = TeethRemoval.removeTeeth(ctrVerts,  target);      
      rerender();
      if(removeT == false) {
         instructions.setText("Need more manual editting!");
         instructions.setForeground (Color.red); 
      }
      else {
         instructions.setText("Teeth removed!");
         instructions.setForeground (Color.gray);  
      }
   }
   
   public void keyPointsSelection() {
      renderKeyVerts(keyVerts, Color.magenta);
      instructions.setText ("Ready for point selection.");
      instructions.setForeground (Color.gray);
    
      SelectionManager mySelectionManager = Main.getMain ().getSelectionManager ();      
      
      mySelectionManager.addSelectionListener (new SelectionListener() {
         public void selectionChanged(SelectionEvent e) {
            ModelComponent mc = e.getLastAddedComponent();
           
            if (mc instanceof Point) {              
               int idx = controlPnts.indexOf ((Point)mc);  
               Vertex3d vert = keyVerts.get (idx);
               
               System.out.println("controlPnt " + idx);
               
               if(selectedKeyVerts.contains (vert)) {
                  RenderProps.setPointColor((Point)mc, Color.magenta);
                  selectedKeyVerts.remove (vert);
               }
               else {
                  if(selectedKeyVerts.size ()<2) {
                     RenderProps.setPointColor((Point)mc, Color.green);
                     selectedKeyVerts.add (vert);      
                  }
               }
               
               instructions.setText ("Number of selected points: "+ selectedKeyVerts.size ());
               instructions.setForeground (Color.gray);
               
               if(selectedKeyVerts.size ()==0) {
                //  feaReroute.setEnabled (false);
                  geoReroute.setEnabled (false);
                  substitute.setEnabled (false);
                  reject.setEnabled (false);
                  pntsRemoval.setEnabled (false);
                  instructions.append ("\nReady for point selection.");
               }
               else if(selectedKeyVerts.size ()==1) {
                //  feaReroute.setEnabled (false);
                  geoReroute.setEnabled (false);
                  substitute.setEnabled (false);
                  reject.setEnabled (false);
                  pntsRemoval.setEnabled (true);
                  instructions.append ("\nOptions: point selection/individual point removal.");
               }
               else if(selectedKeyVerts.size ()==2 && confirmed==true) {
             //     feaReroute.setEnabled (true);
                  geoReroute.setEnabled (true);
                  //substitute.setEnabled (false);
                  //reject.setEnabled (false);
                  pntsRemoval.setEnabled (false);
                  instructions.append ("\nCannot select more than two points at one time.");
                  instructions.append ("\nOptions: point deselection/2-point reroute.");
               }
//               else {
//                  sreroute.setEnabled (false);
//                  pntsRemoval.setEnabled (false);
//                  instructions.append ("\nOptions: point selection/points removal.");
//                  }             
               }
            }
         });      
   }
      
//   public void startFeaReroute() {
//      instructions.setText ("Rerouting...");
//      instructions.setForeground (Color.gray);
//      target.setVertexColoringEnabled();
//      
//      int diff = Math.abs(keyPntsIdx1-keyPntsIdx0);     
//      if(diff > controlPnts.size ()/3 && diff < 2* controlPnts.size ()/3) {
//         instructions.setText ("Reroute range is too large. Please reselect points.");  
//         instructions.setForeground (Color.red);
//      }
//      else {
//         int temp=0;
//         if(keyPntsIdx1<keyPntsIdx0) {
//            temp= keyPntsIdx1;
//            keyPntsIdx1 = keyPntsIdx0;
//            keyPntsIdx0 = temp;
//         }         
//
//         if(keyPntsIdx1-keyPntsIdx0 < controlPnts.size ()/3)
//            vPath = SnakeCurveContourRefinement.snakeVerts (
//               keyVerts.get (keyPntsIdx0),keyVerts.get (keyPntsIdx1), target);
//         else
//            vPath = SnakeCurveContourRefinement.snakeVerts (
//               keyVerts.get (keyPntsIdx1),keyVerts.get (keyPntsIdx0), target);
//
//         if(vPath.size ()==0) {
//            instructions.setText ("Feature reroute failed. "); 
//            instructions.append ("\nPlease reselect points or try shortest-path reroute.");
//            instructions.setForeground (Color.red); 
//            geoReroute.setEnabled (true);
//            feaReroute.setEnabled (false);
//         }
//         else {
//         System.out.println("vPath2 " +vPath.size ()); 
//         
//            for(int k = 0; k< vPath.size()-1; k++) {                  
//             target.setColor(vPath.get(k).getIndex (),Color.pink);                   
//             }
//            rerender();
//            
//            instructions.setText ("Choose whether to accept or reject the newly generated path.");
//            instructions.setForeground (Color.gray);
//            substitute.setEnabled (true);
//            reject.setEnabled (true);
//            geoReroute.setEnabled (false);
//            feaReroute.setEnabled (false);
//            pntsRemoval.setEnabled (false);
//            }  
//      }
//      }
   
   public void startGeoReroute() {
      instructions.setText ("Rerouting...");
      instructions.setForeground (Color.gray);
      target.setVertexColoringEnabled();
      
      int diff = Math.abs(keyPntsIdx1-keyPntsIdx0);     
      if(diff > controlPnts.size ()/3 && diff < 2* controlPnts.size ()/3) {
         instructions.setText ("Reroute range is too large.");  
         instructions.append ("\nPlease reselect points");
         instructions.setForeground (Color.red);
         geoReroute.setEnabled (false);
         //feaReroute.setEnabled (true);
      }
      else {
         int temp=0;
         if(keyPntsIdx1<keyPntsIdx0) {
            temp= keyPntsIdx1;
            keyPntsIdx1 = keyPntsIdx0;
            keyPntsIdx0 = temp;
         }         

         if(keyPntsIdx1-keyPntsIdx0 < controlPnts.size ()/3)
            geoPath = ParallelGeodesicDistance.computeGeodesicDistance (
               keyVerts.get (keyPntsIdx0),keyVerts.get (keyPntsIdx1), target);

         else
            geoPath = ParallelGeodesicDistance.computeGeodesicDistance (
               keyVerts.get (keyPntsIdx1), keyVerts.get (keyPntsIdx0), target);
           
         if(geoPath==null) {
            instructions.setText ("Reroute range is too large. Please reselect points."); 
            instructions.append ("\nNumber of selected points: "+ selectedKeyVerts.size ());
            instructions.setForeground (Color.red);    
            geoReroute.setEnabled (false);
            //feaReroute.setEnabled (false);
         }
         else {

            System.out.println("geoPath " +geoPath.size ()); 
   
            for(int k = 0; k< geoPath.size(); k++) {  
               vPath.add (geoPath.get(k).getVert());
               target.setColor(geoPath.get(k).getVert().getIndex (),Color.pink);                   
            }
         
            rerender();
            
            instructions.setText ("Choose whether to accept or reject the newly generated path.");
            //instructions.append ("\nOptions: route substitution/points selection."); 
            confirmed = false;
            instructions.setForeground (Color.gray);
            substitute.setEnabled (true);
            reject.setEnabled (true);
            geoReroute.setEnabled (false);
         //   feaReroute.setEnabled (false);
            pntsRemoval.setEnabled (false);
            } 
        }
      }
   
   public void substitutePath() {
      
         instructions.setText ("Substituting... ");
         instructions.setForeground (Color.gray);
         int ctrVertIdx0 = ctrVerts.indexOf (keyVerts.get (keyPntsIdx0));
         int ctrVertIdx1 = ctrVerts.indexOf (keyVerts.get (keyPntsIdx1));
         if(keyPntsIdx1-keyPntsIdx0 < controlPnts.size ()/3) {
            
            for(int i= keyPntsIdx1-1; i> keyPntsIdx0; i--) {
               keyVerts.remove (i);
               mech.remove (controlPnts.get (i));
               controlPnts.remove (i);
            }
                        
            for(int j= ctrVertIdx1; j> ctrVertIdx0; j--) {
               target.setColor(ctrVerts.get (j).getIndex (),Color.gray);            
               ctrVerts.remove(j);            
            }
       
            int addkeyVert=0;
            int addKVertCount =0;
           
            for(int k = vPath.size()-1; k>-1; k--) {
               addkeyVert++;
               ctrVerts.add (ctrVertIdx0+1, vPath.get(k));
               target.setColor(vPath.get(k).getIndex (),Color.yellow);
               if(addkeyVert%10==0 && addkeyVert+3 < vPath.size()) {
                  addKVertCount++;
                  keyVerts.add (keyPntsIdx0+1, vPath.get(k));                    
                  Point nPoint = new Point(vPath.get(k).pnt);
                  controlPnts.add(keyPntsIdx0+1, nPoint);
                  mech.add (nPoint);
                  RenderProps.setPointColor(nPoint, Color.magenta);
                  RenderProps.setPointRadius(nPoint, 0.5);
                  RenderProps.setPointStyle(nPoint, PointStyle.SPHERE);
                  RenderProps.setVisible(nPoint, true);               
               }                         
            }
            
            RenderProps.setPointColor(controlPnts.get (keyPntsIdx0), Color.magenta);
            RenderProps.setPointColor(controlPnts.get (keyPntsIdx0+addKVertCount+1), Color.magenta);             
         }
         
         else {
            for(int i= keyVerts.size ()-1; i> keyPntsIdx1; i--) {
               keyVerts.remove (i);
               mech.remove (controlPnts.get (i));
               controlPnts.remove (i);
            }
            for(int i= keyPntsIdx0-1; i> -1; i--) {
               keyVerts.remove (i);
               mech.remove (controlPnts.get (i));
               controlPnts.remove (i);
            }
            for(int j= ctrVerts.size ()-1; j> ctrVertIdx1; j--) {
               target.setColor(ctrVerts.get (j).getIndex (),Color.gray);  
               ctrVerts.remove(j);
            }                             
            for(int j= ctrVertIdx0-1; j> -1; j--) {
               target.setColor(ctrVerts.get (j).getIndex (),Color.gray);  
               ctrVerts.remove(j);
            }
            
            int addkeyVert=0;
            int addKVertCount =0;
            for(int k = 0; k< vPath.size()-1; k++) {
               addkeyVert++;
               ctrVerts.add (vPath.get(k));
               target.setColor(vPath.get(k).getIndex (),Color.yellow);              
               if(addkeyVert%10==0 && addkeyVert+3 < vPath.size()) {
                  addKVertCount++;
                  keyVerts.add (vPath.get(k));                    
                  Point nPoint = new Point(vPath.get(k).pnt);
                  controlPnts.add(nPoint);
                  mech.add (nPoint);
                  RenderProps.setPointColor(nPoint, Color.magenta);
                  RenderProps.setPointRadius(nPoint, 0.5);
                  RenderProps.setPointStyle(nPoint, PointStyle.SPHERE);
                  RenderProps.setVisible(nPoint, true);
                  
               }             
            }
            
            RenderProps.setPointColor(controlPnts.get (0), Color.magenta);
            RenderProps.setPointColor(controlPnts.get (controlPnts.size ()-1-addKVertCount), Color.magenta);              
         } 
         vPath.clear ();      
         selectedKeyVerts.clear();  
         instructions.setText ("Number of selected points:"+ selectedKeyVerts.size ());
         instructions.append ("\nOptions: point selection/teeth removal.");
         instructions.setForeground (Color.gray);
         confirmed =true;
         substitute.setEnabled (false);
         reject.setEnabled (false);
         geoReroute.setEnabled (false);
    //     feaReroute.setEnabled (false);
         pntsRemoval.setEnabled (false);
         rerender();
      
   }         
   
   public void rejectPath() {
     // RenderProps.setPointColor(controlPnts.get (keyPntsIdx0), Color.magenta);
     // RenderProps.setPointColor(controlPnts.get (keyPntsIdx1), Color.magenta);
      for(int k = 0; k< vPath.size()-1; k++) {  
         if(ctrVerts.contains (vPath.get(k))==true)
            target.setColor(vPath.get(k).getIndex (),Color.orange); 
         else
            target.setColor(vPath.get(k).getIndex (),Color.gray); 
      }
      vPath.clear ();          
      //selectedKeyVerts.clear();  
      instructions.setText ("Number of selected points:"+ selectedKeyVerts.size ());
      instructions.append ("\nOptions: point selection/teeth removal.");
      instructions.setForeground (Color.gray);
      confirmed =true;
      substitute.setEnabled (false);
      reject.setEnabled (false);
      geoReroute.setEnabled (true);
 //     feaReroute.setEnabled (true);
      pntsRemoval.setEnabled (false);
      rerender();     
   }
   
   public void removePnts() {
      
         Vertex3d v= selectedKeyVerts.get (0);
         int idxKVert = keyVerts.indexOf (v);
         Vertex3d preKeyV= keyVerts.get ((idxKVert-1 +keyVerts.size ())%keyVerts.size ()); 
         Vertex3d postKeyV= keyVerts.get ((idxKVert+1)%keyVerts.size ());
         keyPntsIdx0= (idxKVert-1 +keyVerts.size ())%keyVerts.size ();
         keyPntsIdx1= (idxKVert+1)%keyVerts.size ();
         
         if((idxKVert-1 +keyVerts.size ())%keyVerts.size ()> (idxKVert+1)%keyVerts.size ()) {
            int s=0;
            while(selectedKeyVerts.contains (preKeyV)) {
               s++;
               selectedKeyVerts.remove (preKeyV);
               preKeyV= keyVerts.get ((idxKVert-1-s+keyVerts.size ())%keyVerts.size ()); 
               keyPntsIdx0= (idxKVert-1-s+keyVerts.size ())%keyVerts.size ();
            }

            int t=0;
            while(selectedKeyVerts.contains (postKeyV)) {
               t++;
               selectedKeyVerts.remove (postKeyV);
               postKeyV= keyVerts.get ((idxKVert+1+t)%keyVerts.size ());  
               keyPntsIdx1= (idxKVert+1+t)%keyVerts.size ();
            }
         }
         else {
            int t=0;
            while(selectedKeyVerts.contains (postKeyV)) {
               t++;
               selectedKeyVerts.remove (postKeyV);
               postKeyV= keyVerts.get ((idxKVert+1+t)%keyVerts.size ());       
               keyPntsIdx1= (idxKVert+1+t)%keyVerts.size ();
            }
            int s=0;
            while(selectedKeyVerts.contains (preKeyV)) {
               s++;
               selectedKeyVerts.remove (preKeyV);
               preKeyV= keyVerts.get ((idxKVert-1-s+keyVerts.size ())%keyVerts.size ());    
               keyPntsIdx0= (idxKVert-1-s+keyVerts.size ())%keyVerts.size ();
            }            
         }
            
         startGeoReroute();
         substitutePath();        
       
      instructions.setText ("Number of selected points:"+ selectedKeyVerts.size ());
      instructions.append ("\nOptions: point selection/teeth removal.");
      instructions.setForeground (Color.gray);
      substitute.setEnabled (false);
      reject.setEnabled (false);
      geoReroute.setEnabled (false);
 //     feaReroute.setEnabled (false);
      pntsRemoval.setEnabled (false);
   }
   
   public boolean getMenuItems(List<Object> items) {
      
      JMenuItem rRemoveItem = GuiUtils.createMenuItem (
         this, "remove teeth", "remove all teeth area");
      items.add (rRemoveItem);
      if(isTeethRemovable)
         rRemoveItem.setEnabled (true);
      else
         rRemoveItem.setEnabled (false);
      
      JMenuItem settingsItem = GuiUtils.createMenuItem (
         this, "reroute settings", "reroute unqualified contour line");
      items.add (settingsItem);
      if(isControlPntsDisplayable)
         settingsItem.setEnabled (true);
      else
         settingsItem.setEnabled (false);
       
      return true;
   }   

   public void actionPerformed (ActionEvent event) {
      if (event.getActionCommand().equals ("remove teeth")) {
         removeTeeth();
      }
      
      if (event.getActionCommand().equals ("reroute settings")) {
         keyPointsSelection();   
         isControlPntsDisplayable = false;
      }
//      if (event.getSource () == feaReroute) { 
//         mouseClicked=true;
//         keyPntsIdx0= keyVerts.indexOf (selectedKeyVerts.get (0));
//         keyPntsIdx1= keyVerts.indexOf (selectedKeyVerts.get (1));
//         startFeaReroute();
//      }
      if (event.getSource () == geoReroute) { 
         mouseClicked=true;
         keyPntsIdx0= keyVerts.indexOf (selectedKeyVerts.get (0));
         keyPntsIdx1= keyVerts.indexOf (selectedKeyVerts.get (1));
         startGeoReroute();
      }
            
      if (event.getSource () == substitute) {    
         mouseClicked=true;
         substitutePath();
      }
      
      if (event.getSource () == reject) {    
         mouseClicked=true;
         rejectPath();
       
      }
      
      if (event.getSource () == pntsRemoval) { 
         mouseClicked=true;
         removePnts();
         
      }
      
//      if (event.getSource () == undoTR) { 
//         mouseClicked=true;
//         undoTeethRemoval();
//         
//      }
      
   } 
}