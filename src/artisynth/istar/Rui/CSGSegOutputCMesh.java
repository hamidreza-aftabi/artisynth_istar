 package artisynth.istar.Rui;
//Rui: from artisynth_models_registration
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JMenuItem;

import artisynth.core.driver.Main;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemMeshComp;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.editorManager.EditorUtils;
import artisynth.core.gui.selectionManager.ClassFilter;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.util.ExtensionFileFilter;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.core.femmodels.EmbeddedFem;

import artisynth.models.registration.DynamicRegistrationController;
import artisynth.models.registration.correspondences.GMMMeshCorrespondence;
import maspack.collision.IntersectionContour;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.ICPRegistration;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.geometry.io.GenericMeshWriter;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.Renderer.FaceStyle;
import maspack.widgets.GuiUtils;

public class CSGSegOutputCMesh extends RootModel {
   FemMeshComp source;
   PolygonalMesh cMesh; //contour mesh
   PolygonalMesh sourcePolyMesh;
   FemMeshComp cMeshComp;
   PolygonalMesh target;
   FixedMeshBody fixedTarget;
   FixedMeshBody fixedCMesh;
   FixedMeshBody fixedSmoothedMesh;
   FemModel3d fem;
   Point3d targetTr = new Point3d(); 
   double runTime = 1;  //1 for 10k-20k-vertex meshes
   double fDensity =0.0001; //0.0001 for 10k-20k-vertex meshes
   double fScaling = 1e6;  //1e6 for 10k-20k-vertex meshes
   Intersector myIntersector; 
   ArrayList<IntersectionContour> myContours = new ArrayList<IntersectionContour>();
   MechModel mech = new MechModel("mech");
   long veryStartTime=System.nanoTime(); 
   String fileDir;
   String modelName;
  
   @Override
   public void build(String[] args) throws IOException {                         
      super.build(args);     
      addModel(mech);     
      mech.setGravity(0, 0, 0);  // turn off gravity for registration
      
      String meshDirectory = ArtisynthPath.getSrcRelativePath(this, "mandible/");
      String sourceMeshName = "mandible_3kv.stl";    
      String cMeshName = "16clip.stl"; 
         
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
      mech.add (fixedCMesh);
      
      RenderProps.setVisible (fem.getNodes (), false);
      RenderProps.setVisible (fem.getElements (), false);
      RenderProps.setVisible (source, false);     
      RenderProps.setVisible (cMeshComp, false); 
      RenderProps.setAlpha (fixedCMesh, 0.2); //0.2
      RenderProps.setFaceStyle (fixedCMesh, FaceStyle.NONE);
      RenderProps.setDrawEdges (fixedCMesh, true);
      RenderProps.setEdgeColor (fixedCMesh, Color.cyan);
   }
   
   public void startRegistration(){
      ICPRegistration myRegister = new ICPRegistration();
      AffineTransform3d X = new AffineTransform3d();
      myRegister.registerICP (X, target, sourcePolyMesh, 3, 6);
      fem.transformGeometry(X);

      myIntersector = new Intersector (target, cMesh, 0, Color.red);
      myContours = myIntersector.getContours ();
      mech.addRenderable (myIntersector);
     
      DynamicRegistrationController controller = new DynamicRegistrationController (mech);      
      GMMMeshCorrespondence gmm = new GMMMeshCorrespondence ();
      gmm.setNoiseFraction (0.01); //omega
      gmm.setNearestK (10);  // consider 10 nearest points
      
      controller.addRegistrationTarget (source, target, 1.0, gmm);
      controller.setForceScaling (fScaling);    //beta    
      controller.setName("registration");
      addController(controller);
  //    addController (new TimeCheck());
        
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
      Main.getMain ().setTimelineVisible (false);
      GLViewer viewer = driver.getViewer ();
      if (viewer != null) {
         viewer.setBackgroundColor (Color.WHITE);
         viewer.setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      }   
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
   
   private void contourProcessing() {
      long startTime = System.nanoTime();    
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
      
      fem.removeMeshComp (cMeshComp);
      mech.remove (cMeshComp);
      ArrayList<ArrayList<Vertex3d>> keyVerts = new ArrayList<ArrayList<Vertex3d>>();
      myContours = myIntersector.getContours ();
      CSGContourProcess.projection (myContours, keyVerts, target, cMesh, 45);      
    
      for(int i=0; i<keyVerts.size (); i++) {   
//         System.out.println("Contour "+i+" has " + keyVerts.get (i).size ()+" control points");
//         System.out.println("   Checking sdf... "); 
         keyVerts.set (i, CSGContourProcess.checkSDF (keyVerts.get (i), target)); 
         System.out.println("   Running snake... ");
         for(int t=0; t<3; t++) {
            keyVerts.set (i, CSGContourProcess.snakeCurv(keyVerts.get (i), target)); 
         }    
      }
     
      CSGContourProcess.gaussianDeform(keyVerts, cMesh);
      cMesh.notifyVertexPositionsModified();
      rerender(); 
      System.out.println( modelName +" completed! ");
      
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
      RenderProps.setVisible (fixedCMesh, true); 

      String cMeshName = ArtisynthPath.getSrcRelativePath(this, 
         "mandible/savedCMesh/"+modelName.substring(0, modelName.length() - 4)+"_CMesh.stl"); 
         
      try {
         File file = new File (cMeshName);
         GenericMeshWriter writer = new GenericMeshWriter(file);
         writer.writeMesh(cMesh);
         writer.close();
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }
    
   public void rebuild() {
      removeModel (mech);
      mech = null;
      fem.clear ();
      removeAllControllers ();

      clearRenderables ();    
      Main.getMain ().getSelectionManager ().clearSelections ();
      Main.getMain ().getMainFrame ().getSelectCompPanelHandler().clear();
      Main.getMain ().reset ();
     
      ClassFilter myClassFilter = new ClassFilter (Point.class);
      Main.getMain ().getSelectionManager ().addFilter (myClassFilter);
      Main.getMain ().getSelectionManager ().filterSelections (myClassFilter);
      
      mech = new MechModel("mech");
      mech.setGravity (0, 0, 0);
      addModel(mech);
  
      String meshDirectory = ArtisynthPath.getSrcRelativePath(this, "mandible/");
      String sourceMeshName = "mandible_3kv.stl";    
      String cMeshName = "16clip.stl"; // contour mesh
         
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
      mech.add (fixedCMesh);
      
      RenderProps.setVisible (fem.getNodes (), false);
      RenderProps.setVisible (fem.getElements (), false);
      RenderProps.setVisible (source, false);     
      RenderProps.setVisible (cMeshComp, false); 
      RenderProps.setAlpha (fixedCMesh, 0.2); //0.2
      RenderProps.setFaceStyle (fixedCMesh, FaceStyle.NONE);
      RenderProps.setDrawEdges (fixedCMesh, true);
      RenderProps.setEdgeColor (fixedCMesh, Color.cyan);
      
      myIntersector=null;
      myContours.clear ();      
 
   }

   public void loadTarget() {             
      System.out.println("Loading mesh " + modelName + "...");
      target = GenericModel.loadGeometry(fileDir+ "/", modelName);
      
      targetTr = transformMeshOrigin(target);
      fixedTarget = new FixedMeshBody("target", target);
      mech.addMeshBody(fixedTarget);
      RenderProps.setAlpha (fixedTarget, 1);
      RenderProps.setFaceColor (fixedTarget, new Color (1f, 1f, 0.8f)); 

   }
    
   public boolean getMenuItems(List<Object> items) {
      
      JMenuItem loadingItem = GuiUtils.createMenuItem (
         this, "Load mandibles", "");
      items.add (loadingItem);
    
      return true;
   }   

   public void actionPerformed (ActionEvent event) {  
      if (event.getActionCommand().equals("Load mandibles")) {
         
         Main main = Main.getMain();
         JFrame frame = main.getMainFrame();
         String initialFileDir = ArtisynthPath.getSrcRelativePath(this, "mandible/");
         JFileChooser chooser = new JFileChooser(initialFileDir);
         ExtensionFileFilter filter = new ExtensionFileFilter ("OBJ, STL, PLY files", "obj", "stl", "ply");
         chooser.setFileFilter (filter);
         int retVal = chooser.showOpenDialog (frame);  
         
         if (retVal == JFileChooser.APPROVE_OPTION) {
            fileDir=chooser.getCurrentDirectory().getPath ();          
            File[] directoryListing = chooser.getCurrentDirectory().listFiles();  
            for (File child : directoryListing) {         
               if(child.getName().endsWith (".stl") ||
               child.getName().endsWith (".obj") ||
               child.getName().endsWith (".ply")) { 
                  modelName =child.getName ();
                  if (fixedTarget!= null) 
                     rebuild();
                  loadTarget();
                  
                  BufferedWriter out = null;
                  try {
                     FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
                        "timer/regtime.txt"), true); 
                     out = new BufferedWriter(fstream);
                     out.write("\n"+ modelName +" ");
                     out.close ();
                  }
                  catch (IOException e) {
                     System.err.println("Error: " + e.getMessage());
                  }

                  try {
                     FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
                        "timer/snaketime.txt"), true); 
                     out = new BufferedWriter(fstream);
                     out.write("\n"+ modelName +" ");
                     out.close ();
                  }
                  catch (IOException e) {
                     System.err.println("Error: " + e.getMessage());
                  }
                  veryStartTime = System.nanoTime(); 
                  startRegistration();   
                  Main.getMain().play(runTime);
                  Main.getMain().waitForStop();
                  contourProcessing();
                  
                  } 
               }
            
           
            }                    
         }   
      } 
}
