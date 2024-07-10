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
import maspack.matrix.Vector3d;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;

public class mandibleAnimation extends RootModel {
   String meshDirectory = ArtisynthPath.getSrcRelativePath(this, "mandible/");  
   PolygonalMesh ntMesh;
   PolygonalMesh originalMesh;
   FixedMeshBody originalMeshBody;
   FixedMeshBody ntMeshBody;
   Vector3d ntCentroid = new Vector3d();
   Vector3d originalCentroid = new Vector3d();
   
   @SuppressWarnings("deprecation")
   public void build(String[] args) throws IOException {
      
      MechModel mech = new MechModel("mech");
      addModel(mech);
            
      // turn off gravity for registration
      mech.setGravity(0, 0, 0);
      String originalMeshName = "proposedT48/049-Mandible.stl";//"autoNTResult/049.stl";
      System.out.println("Loading mesh " + originalMeshName + "...");
      originalMesh = GenericModel.loadGeometry(this.meshDirectory, originalMeshName);
      originalCentroid=transformMeshOrigin(originalMesh);
      originalMeshBody = new FixedMeshBody("original_mesh", originalMesh);      
      mech.addMeshBody(originalMeshBody);
      RenderProps.setFaceColor (originalMeshBody, new Color (1f, 1f, 0.8f));   //bone  
      
//      String ntMeshName ="autoToothRemoval/049.stl";
//      System.out.println("Loading mesh " + ntMeshName + "...");
//      ntMesh = GenericModel.loadGeometry(this.meshDirectory, ntMeshName);
//      transformMeshOrigin(ntMesh);
//      ntMesh.translate(new Vector3d(-150,0,0));
//      ntMeshBody = new FixedMeshBody("nt_mesh", ntMesh);      
//      mech.addMeshBody(ntMeshBody);
//      RenderProps.setFaceColor (ntMeshBody, new Color (1f, 1f, 0.8f));   //bone        

   }
   
   public void prerender (RenderList rlist) {            
      originalMesh.XMeshToWorld.mulRotZ (2*Math.PI/180);      
     // ntMesh.XMeshToWorld.mulRotZ (2*Math.PI/180);
      super.prerender(rlist);  
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
 
  
   // Rui: move the center of mesh to origin, align
   public static Point3d transformMeshOrigin(PolygonalMesh mesh){
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