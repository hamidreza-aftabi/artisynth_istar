package artisynth.istar.Rui;

import java.awt.Color;
//import java.awt.Polygon;
import java.io.IOException;
import java.util.ArrayList;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;

public class cleanmesh extends RootModel {
   String meshDirectory = ArtisynthPath.getSrcRelativePath(this, "mandible/");  
   
   @SuppressWarnings("deprecation")
   public void build(String[] args) throws IOException {
      
      MechModel mech = new MechModel("mech");
      addModel(mech);
            
      // turn off gravity for registration
      mech.setGravity(0, 0, 0);
      
  //    String meshDirectory = "/Users/GladysYang/eclipse_workspace/artisynth_projects/src/artisynth/models/Rui/mandible/NT/";
      String meshDirectory = ArtisynthPath.getSrcRelativePath(this, "mandible/");
      String meshName = "proposedT48/025-Mandible.stl";
//      File dir = new File(meshDirectory);
//      File[] directoryListing = dir.listFiles();  
//      for (File child : directoryListing) {
//         
//         if(child.getName ().endsWith (".stl")) {
//            meshName = child.getName (); 
//            // add a target mesh (scaled to have same volume)
//            System.out.println("Loading mesh " + meshName + "...");
//            PolygonalMesh mesh0 = GenericModel.loadGeometry(meshDirectory, meshName);
//            if(mesh0.isWatertight ()==false)
//              System.out.println("not water tight");
//         }
//         }
      
      // add a target mesh (scaled to have same volume)
      System.out.println("Loading mesh " + meshName + "...");
      PolygonalMesh originalMesh = GenericModel.loadGeometry(this.meshDirectory, meshName);
      if(originalMesh.isWatertight ())
        System.out.println("water tight");
      fixMesh(originalMesh);
      System.out.println("finish cleaning");
      transformMeshOrigin(originalMesh);
      FixedMeshBody originalMeshBody = new FixedMeshBody("target_mesh", originalMesh);      
      mech.addMeshBody(originalMeshBody);
            
//      RenderProps.setAlpha(originalMeshBody, 0.8); //0.9
      ////RenderProps.setFaceColor(originalMeshBody, Color.CYAN);
      
     
   }
   
   @Override
   public void attach (DriverInterface driver) {
      super.attach (driver);
      
      GLViewer viewer = driver.getViewer ();
      if (viewer != null) {
         viewer.setBackgroundColor (Color.WHITE);
         viewer.setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      }
   }
   
   private void fixMesh(PolygonalMesh mesh) {
      if(mesh.isClosed ()) {
         System.out.println("closed");
      }
      else
      {
         System.out.println("open");
      }
      //   merge vertices 1e-10
         System.out.println("merge vertices");
         mesh.mergeCloseVertices (100e-10);

       //  remove detached pieces
         System.out.println("remove disconnected faces");
         mesh.removeDisconnectedFaces ();
         System.out.println("remove disconnected vertices");
         mesh.removeDisconnectedVertices ();
         
         if(mesh.isClosed ()) {
            System.out.println("closed");
         }
         else
         {
            System.out.println("open");
         }
         

        // reduce size 8000
        // System.out.println("reduce size");
       // MeshUtilities.quadricEdgeCollapse (mesh, mesh.numVertices () - 100000);
        
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
   
  
   
}