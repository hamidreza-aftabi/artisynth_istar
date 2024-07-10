package artisynth.istar.Rui;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
//import java.awt.Polygon;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;

import artisynth.core.mechmodels.MechModel;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.render.GL.GLViewer;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.util.BinaryHeap;

//compare surface distances between the proposed segmentation and segmentation using 3DSlicer
public class AccuracyEvaluation2 extends RootModel {
   //The input for the proposed segmentation (models with teeth)
   String meshDirectory1 = ArtisynthPath.getSrcRelativePath(this, "mandible/proposedT48/");
   // The result of the proposed segmentation
   String meshDirectory2 = ArtisynthPath.getSrcRelativePath(this, "mandible/proposedNT48Result/");
   // The meshes directly exported from the dentulous CT scans
   String meshDirectory3 = ArtisynthPath.getSrcRelativePath(this, "mandible/3DSlicerT48/");
   // The result of the segmentation using 3DSlicer
   String meshDirectory4 = ArtisynthPath.getSrcRelativePath(this, "mandible/3DSlicerNT48Result/");
   
   public void build(String[] args) throws IOException {
      super.build(args);
      MechModel mech = new MechModel("mech");
      addModel(mech);
      
      File dir = new File(meshDirectory3);
      File[] directoryListing = dir.listFiles();
      for (File child : directoryListing) {         
         if(child.getName().endsWith (".stl") ||
         child.getName().endsWith (".obj") ||
         child.getName().endsWith (".ply")) { 
            String modelName =child.getName ();
         
            // add a target mesh (scaled to have same volume)
            System.out.println("Loading mesh " + modelName + "...");
            PolygonalMesh mesh1 = GenericModel.loadGeometry(this.meshDirectory1, modelName);           
            PolygonalMesh mesh2 = GenericModel.loadGeometry(this.meshDirectory2, modelName.substring (0, 3) +".stl");
            PolygonalMesh mesh3 = GenericModel.loadGeometry(this.meshDirectory3, modelName);           
            PolygonalMesh mesh4 = GenericModel.loadGeometry(this.meshDirectory4, modelName.substring (0, 3) +".stl");
            
           
//            PolygonalMesh mesh1 = GenericModel.loadGeometry(this.meshDirectory1, modelName1); //wt          
//            PolygonalMesh mesh2 = GenericModel.loadGeometry(this.meshDirectory2, modelName2);  //nt
//            PolygonalMesh mesh3 = GenericModel.loadGeometry(this.meshDirectory3, modelName3); //wt          
//            PolygonalMesh mesh4 = GenericModel.loadGeometry(this.meshDirectory4, modelName4);  //nt
          
            ArrayList<Vertex3d> cutSurfaceV1 = new ArrayList<Vertex3d>();
            ArrayList<Vertex3d> cutSurfaceV2 = new ArrayList<Vertex3d>();
           
            mesh2.setVertexColoringEnabled ();
            mesh4.setVertexColoringEnabled ();
            BVFeatureQuery query = new BVFeatureQuery();             
            for (Vertex3d v:mesh2.getVertices ()) {                        
               Vertex3d nearestV=  query.nearestVertexToPoint (mesh1, v.getPosition ());   
               if(v.distance (nearestV)>0.0001) {               
                  mesh2.setColor (v.getIndex (), Color.pink);          
                  cutSurfaceV1.add (v);
               }
            }
          
            for (Vertex3d v:mesh4.getVertices ()) {                        
               Vertex3d nearestV=  query.nearestVertexToPoint (mesh3, v.getPosition ());   
               if(v.distance (nearestV)>0.0001) {               
                  mesh4.setColor (v.getIndex (), Color.pink);          
                  cutSurfaceV2.add (v);
               }
            }
    
//            FixedMeshBody fixedmesh2 = new FixedMeshBody("mesh2", mesh2);
//            mech.addMeshBody(fixedmesh2);
//            FixedMeshBody fixedmesh4 = new FixedMeshBody("mesh4", mesh4);
//            mech.addMeshBody(fixedmesh4);
                  
            BinaryHeap<Double> distHeap1 = computeDistHeap(cutSurfaceV1,mesh2,mesh4);
            BinaryHeap<Double> distHeap2 = computeDistHeap(cutSurfaceV2,mesh4,mesh2);
            double symRootMeanSquare =0;
            double symHausdorff95 =0;
            if(distHeap1.size ()!=0) {
               // Symmetric mean square distance               
               symRootMeanSquare = computeSymRMSD(distHeap1, distHeap2);
               System.out.println("Symmetric root mean square distance:" + symRootMeanSquare);
               
               // Symmetric Hausdorff 95 distance              
               symHausdorff95 = computeSymHausdorff95(distHeap1, distHeap2);
               System.out.println("Symmetric Hausdorff 95 distance:" + symHausdorff95);
            }    
            
            
            BufferedWriter out = null;
            try {
               FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
                  "evaluation/accuracy2.txt"), true); 
               out = new BufferedWriter(fstream);
               out.write("\n"+ modelName +" ");
               out.write(" "+ symRootMeanSquare +" ");
               out.write(" "+ symHausdorff95 +" ");
               out.close ();
            }
            catch (IOException e) {
               System.err.println("Error: " + e.getMessage());
            }
    
         }
      
      }
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
   
   public static double computeSymHausdorff95(BinaryHeap<Double> distHeap1, BinaryHeap<Double> distHeap2) {
      double symHausdorff95 = Math.max(computeHausdorff95(distHeap1), computeHausdorff95(distHeap2));     
      return symHausdorff95;
   }
   
   public static double computeSymRMSD(BinaryHeap<Double> distHeap1, BinaryHeap<Double> distHeap2) {
      double sum1 = 0.0;
      double sum2 = 0.0;
      
      Iterator<Double> it1 = distHeap1.iterator();
      while (it1.hasNext()) {
         double d = it1.next();
         sum1 += d*d;
      }
      
      Iterator<Double> it2 = distHeap2.iterator();
      while (it2.hasNext()) {
         double d = it2.next();
         sum2 += d*d;
      }
      
      double rmsd = Math.sqrt ((sum1+sum2) / (distHeap1.size()+distHeap2.size()));
      return rmsd;
   }
   
   public static double computeHausdorff95(BinaryHeap<Double> distHeap) {          
      int number95 = (int)(0.05 * distHeap.size());
      
      for(int i = 0; i < number95; i++) 
         distHeap.poll();  
         
      double hausdorff95 = distHeap.poll(); //pollLast();
           
      return hausdorff95;
   }
   
   public static BinaryHeap<Double> computeDistHeap(
      ArrayList<Vertex3d> cut1, PolygonalMesh mesh1, PolygonalMesh mesh2) {
      BinaryHeap<Double> distHeap = new BinaryHeap<Double>(20000, null, false);  //max-heap
      
      int m=0;
      
      for (Vertex3d v : cut1) {
         Point3d q = new Point3d();          
         double minDist = BVFeatureQuery.distanceToMesh (q, mesh2, v.getPosition ()); 
         // q is the closest point on mesh2 to v  
         
         distHeap.add(minDist);   
         m++;
        
      }
      
      System.out.println("m " + m);
      System.out.println("distHeap " + distHeap.size ());
      return distHeap;
      
   }


}
   
  
