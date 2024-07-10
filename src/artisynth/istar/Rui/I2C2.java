package artisynth.istar.Rui;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
//import java.awt.Polygon;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

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

//https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3869880/

public class I2C2 extends RootModel {
    //calculate the intraclass correlation coefficient between two rounds of segmentation using 3DSlicer
   
   // The meshes directly exported from the dentulous CT scans
   String meshDirectory0 = ArtisynthPath.getSrcRelativePath(this, "mandible/3DSlicerT48/");
   // The result of the segmentation using 3DSlicer
   String meshDirectory1 = ArtisynthPath.getSrcRelativePath(this, "mandible/3DSlicerNT48Result/");
   // The result of the 2nd round of segmentation using 3DSlicer
   String meshDirectory2 = ArtisynthPath.getSrcRelativePath(this, "mandible/3DSlicerNT10Result/");
       
   public void build(String[] args) throws IOException {
      super.build(args);
      MechModel mech = new MechModel("mech");
      addModel(mech);
      
      double mat[][] = new double[20][5000]; //20
      double[] V_ave_all = new double[5000];
      double trace_w =0;
      double trace_u =0;
      
      int fileCount=0;
      File dir = new File(meshDirectory2);
      File[] directoryListing = dir.listFiles();
      for (File child : directoryListing) {         
         if(child.getName().endsWith (".stl") ||
         child.getName().endsWith (".obj") ||
         child.getName().endsWith (".ply")) { 
            String modelName =child.getName ();
         
            // add a target mesh (scaled to have same volume)
            System.out.println("Loading mesh " + modelName + "...");
            PolygonalMesh mesh0 = GenericModel.loadGeometry(this.meshDirectory0, modelName.substring (0, 3) +"-Mandible.stl");           
            PolygonalMesh mesh1 = GenericModel.loadGeometry(this.meshDirectory1, modelName);
            PolygonalMesh mesh2 = GenericModel.loadGeometry(this.meshDirectory2, modelName);           
            
            ArrayList<Vertex3d> cutSurfaceV1 = new ArrayList<Vertex3d>();
            ArrayList<Vertex3d> cutSurfaceV2 = new ArrayList<Vertex3d>();
          
            mesh1.setVertexColoringEnabled ();
            mesh2.setVertexColoringEnabled ();
            BVFeatureQuery query = new BVFeatureQuery();             
            for (Vertex3d v:mesh1.getVertices ()) {                        
               Vertex3d nearestV=  query.nearestVertexToPoint (mesh0, v.getPosition ());   
               if(v.distance (nearestV)>0.0001) {               
                  mesh1.setColor (v.getIndex (), Color.pink);          
                  cutSurfaceV1.add (v);
               }
            }
          
            for (Vertex3d v:mesh2.getVertices ()) {                        
               Vertex3d nearestV=  query.nearestVertexToPoint (mesh0, v.getPosition ());   
               if(v.distance (nearestV)>0.0001) {               
                  mesh2.setColor (v.getIndex (), Color.pink);          
                  cutSurfaceV2.add (v);
               }
            }
            
            System.out.println("cutSurfaceV1 " + cutSurfaceV1.size ());
            System.out.println("cutSurfaceV2 " + cutSurfaceV2.size ());
    
//            FixedMeshBody fixedmesh1 = new FixedMeshBody("mesh1", mesh1);
//            mech.addMeshBody(fixedmesh1);
//            FixedMeshBody fixedmesh2 = new FixedMeshBody("mesh2", mesh2);
//            mech.addMeshBody(fixedmesh2);
            

            // ICC
            
            List<Integer> numbers = new ArrayList<>();
            for (int i = 0; i < cutSurfaceV1.size (); i++) {
               if(i<5000)
                  numbers.add(1);
               else
                  numbers.add(0);
            }
            Collections.shuffle(numbers);
            
            double[] V1 = new double[5000];
            double[] V2 = new double[5000];
            double[] V_ave_sub = new double[5000];
            int count=0;
            for(int i = 0; i < cutSurfaceV1.size (); i++) {
               Vertex3d v= cutSurfaceV1.get (i);
               if(numbers.get (i)==1) {
                  V1[count]=v.pnt.z;
                  
//                  Line ray = new Line(v.pnt, new Vector3d (0, 0, 1)); 
//                  Point3d pos1 = new Point3d();                
//                  query.nearestFaceAlongLine (pos1, null, mesh2, v.pnt, ray.getDirection (),10,10);                  
//                  V2.add (pos1.z);
                  
                  double minDist=100;
                  Vertex3d nearestV = new Vertex3d();
                  for(int j = 0; j < cutSurfaceV2.size (); j++) {
                     Vertex3d vTemp= cutSurfaceV2.get (j);
                     double dist= Math.sqrt (Math.pow (v.pnt.x-vTemp.pnt.x, 2)+Math.pow (v.pnt.y-vTemp.pnt.y, 2));
                     if(dist<minDist) {
                        minDist = dist;
                        nearestV = vTemp;
                     }
                  }
                  V2[count]=nearestV.pnt.z;
                  V_ave_sub[count] = (V1[count] + V2[count])/2;
                  trace_u += (V1[count]-V_ave_sub[count])* (V1[count]-V_ave_sub[count]);
                  trace_u += (V2[count]-V_ave_sub[count])* (V2[count]-V_ave_sub[count]);
                  
                  V_ave_all[count] += (V1[count] + V2[count])/2;
                  count++;
                  if(count==10) //10
                     V_ave_all[count] /= 10; //10
                 
               }
            }
             
            mat[2*fileCount]=V1;
            mat[2*fileCount+1]=V2;
        
            
            // MSD HD95                
            BinaryHeap<Double> distHeap1 = computeDistHeap(cutSurfaceV1,mesh1,mesh2);
            BinaryHeap<Double> distHeap2 = computeDistHeap(cutSurfaceV2,mesh2,mesh1);
            double symRootMeanSquare =0;
            double symHausdorff95 =0;
            if(distHeap1.size ()!=0) {
               // Symmetric mean square distance               
               symRootMeanSquare = computeSymRMSD(distHeap1, distHeap2);
               System.out.println("Symmetric mean square distance:" + symRootMeanSquare);
               
               // Symmetric Hausdorff 95 distance              
               symHausdorff95 = computeSymHausdorff95(distHeap1, distHeap2);
               System.out.println("Symmetric Hausdorff 95 distance:" + symHausdorff95);
            }    
            
            
            BufferedWriter out = null;
            try {
               FileWriter fstream = new FileWriter(ArtisynthPath.getSrcRelativePath(this, 
                  "mandible/evaluation/accuracy10.txt"), true); 
               out = new BufferedWriter(fstream);
               out.write("\n"+ modelName +" ");
               out.write(" "+ symRootMeanSquare +" ");
               out.write(" "+ symHausdorff95 +" ");
               out.close ();
            }
            catch (IOException e) {
               System.err.println("Error: " + e.getMessage());
            }
    
            fileCount++;
         }
      }
      
      trace_u /= 10; //10
      
      for(int i=0; i< mat.length; i++) {
         for(int j=0; j< 5000; j++) {
            trace_w += (mat[1][j]-V_ave_all[j])* (mat[1][j]-V_ave_all[j]);                     
         }
      }
      
      trace_w /= 19; //19
      
      double result = 1- trace_u/trace_w;
      System.out.println("trace_u " + trace_u);
      System.out.println("trace_w " + trace_w);
      System.out.println("result " + result);
      
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
   
  
