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

//compare surface distances between the proposed segmentation and automatic segmentation
public class AccuracyEvaluation1 extends RootModel {
   String meshDirectory1 = ArtisynthPath.getSrcRelativePath(this, "mandible/autoNTResult/");
   // The result of the proposed segmentation
   String meshDirectory2 = ArtisynthPath.getSrcRelativePath(this, "mandible/proposedNT48Result/"); 
   //The input for the proposed segmentation (models with teeth)
   String meshDirectory3 = ArtisynthPath.getSrcRelativePath(this, "mandible/proposedT48Result/"); 

   public void build(String[] args) throws IOException {
      super.build(args);
      MechModel mech = new MechModel("mech");
      addModel(mech);
      
      File dir = new File(meshDirectory1);
      File[] directoryListing = dir.listFiles();
      for (File child : directoryListing) {         
         if(child.getName().endsWith (".stl") ||
         child.getName().endsWith (".obj") ||
         child.getName().endsWith (".ply")) { 
            String modelName =child.getName ();
         
            // add a target mesh (scaled to have same volume)
            System.out.println("Loading mesh " + modelName + "...");
            PolygonalMesh mesh1 = GenericModel.loadGeometry(this.meshDirectory1, modelName);        
            PolygonalMesh mesh2 = GenericModel.loadGeometry(this.meshDirectory2, modelName);
            PolygonalMesh mesh3 = GenericModel.loadGeometry(this.meshDirectory3, modelName.substring (0, 3) +"-Mandible.stl");
           
            
            // dice coefficient
      //      CSG dist = new CSG();
      //      double dice = dist.computeDice(mesh1, mesh2);      
      //      System.out.println("dice coefficient:" + dice);
            
            ArrayList<Vertex3d> cutSurfaceV1 = new ArrayList<Vertex3d>();
            ArrayList<Vertex3d> cutSurfaceV2 = new ArrayList<Vertex3d>();
           
            mesh1.setVertexColoringEnabled ();
            mesh2.setVertexColoringEnabled ();
            BVFeatureQuery query = new BVFeatureQuery();             
            for (Vertex3d v:mesh1.getVertices ()) {                        
               Vertex3d nearestV=  query.nearestVertexToPoint (mesh3, v.getPosition ());   
               if(v.distance (nearestV)>0.000001) {               
                  mesh1.setColor (v.getIndex (), Color.pink);          
                  cutSurfaceV1.add (v);
               }
            }
          
            for (Vertex3d v:mesh2.getVertices ()) {                        
               Vertex3d nearestV=  query.nearestVertexToPoint (mesh3, v.getPosition ());   
               if(v.distance (nearestV)>0.000001) {               
                  mesh2.setColor (v.getIndex (), Color.pink);          
                  cutSurfaceV2.add (v);
               }
            }
            
            mesh1.setVertexColoringEnabled ();
            mesh2.setVertexColoringEnabled ();
            BinaryHeap<Double> distHeap1 = computeDistHeap(cutSurfaceV1,mesh1,mesh2);
            BinaryHeap<Double> distHeap2 = computeDistHeap(cutSurfaceV2,mesh1,mesh2);
            double symRootMeanSquare =0;
            double symHausdorff95 =0;
            if(distHeap1.size ()!=0) {
               // Symmetrical mean square distance 
               
               symRootMeanSquare = computeSymRMSD(distHeap1, distHeap2);
               System.out.println("Symmetric root mean square distance:" + symRootMeanSquare);
               
               // Symmetrical Hausdorff 95 distance
               
               symHausdorff95 = computeSymHausdorff95(distHeap1, distHeap2);
               System.out.println("Symmetric Hausdorff 95 distance:" + symHausdorff95);
            }    
//            FixedMeshBody fixedmesh1 = new FixedMeshBody("mesh1", mesh1);
//            mech.addMeshBody(fixedmesh1);      
//            FixedMeshBody fixedmesh2 = new FixedMeshBody("mesh2", mesh2);
//            mech.addMeshBody(fixedmesh2);
//            RenderProps.setVisible (fixedmesh2, false);
            
            BufferedWriter out = null;
            try {
               FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
                  "evaluation/accuracy1.txt"), true); 
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
   
//   public static BinaryHeap<Double> computeDistHeap(PolygonalMesh mesh1, PolygonalMesh mesh2) {
//      BinaryHeap<Double> distHeap = new BinaryHeap<Double>(2000, null, false);  //max-heap
//      BVFeatureQuery query = new BVFeatureQuery();  
//   
//      int m=0;
////      //l vs auto
////      for (Vertex3d v : mesh1.getVertices()) {
////        
////         Point3d q = new Point3d();        
////         Vertex3d nearestV= query.nearestVertexToPoint (mesh2, v.getPosition ());
////       
////         if(nearestV.distance (v)>0.000001) {
////            mesh1.setColor (v.getIndex (), Color.pink);
////            double minDist = BVFeatureQuery.distanceToMesh (q, mesh2, v.getPosition ()); 
////            // q is the closest point on mesh2 to v           
////            distHeap.add(minDist);   
////            m++;
////         }
////       }
//      
//      //a vs l
//      
//      for (Vertex3d v : mesh1.getVertices()) {
//         Point3d q = new Point3d();          
//         double minDist = BVFeatureQuery.distanceToMesh (q, mesh2, v.getPosition ()); 
//         // q is the closest point on mesh2 to v  
//         if(minDist>0.1) {
//         distHeap.add(minDist);   
//         mesh1.setColor (v.getIndex (), Color.pink);
//         m++;
//         }
//      }
//      
////      System.out.println("m " + m);
////      System.out.println("distHeap " + distHeap.size ());
//      return distHeap;
//      
//   }

}
   
  
