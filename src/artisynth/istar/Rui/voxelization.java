package artisynth.istar.Rui;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
//import java.awt.Polygon;
import java.io.IOException;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;

public class voxelization extends RootModel {
    
   
   public void build(String[] args) throws IOException {
      
      MechModel mech = new MechModel("mech");
      addModel(mech);
            
      // turn off gravity for registration
      mech.setGravity(0, 0, 0);
      
      String meshDirectory = "/Users/GladysYang/voxelization/added_NT/";
      String meshName;
      File dir = new File(meshDirectory);
      File[] directoryListing = dir.listFiles();  
      for (File child : directoryListing) {
         
         if(child.getName ().endsWith (".stl")) {
            meshName = child.getName (); 
            // add a target mesh (scaled to have same volume)
            System.out.println("Loading mesh " + meshName + "...");
            PolygonalMesh mesh0 = GenericModel.loadGeometry(meshDirectory, meshName);
            if(mesh0.isWatertight ())
              System.out.println("water tight");
               
            double inf = Double.POSITIVE_INFINITY;
            Point3d max = new Point3d (-inf, -inf, -inf);
            Point3d min = new Point3d ( inf,  inf,  inf);
            mesh0.updateBounds (min, max);
            int width = (int) (max.x -min.x);
            int height = (int) (max.y -min.y);
            int length = (int) (max.z -min.z);
            
            System.out.println("voxelizing...");
            BufferedWriter outputWriter = null;
            outputWriter = new BufferedWriter(new FileWriter(
               "/Users/GladysYang/txt_npy/txt_addednt/" + meshName.substring (0, meshName.length ()-4) + ".txt"));
            outputWriter.write(width + " " + height + " " + length + "\n");
            
         
            for(int i=0; i<width; i++) {          
               for(int j=0; j<height; j++) {
                  for(int k=0; k<length; k++) {
                     Point3d p = new Point3d(min.x+i+0.5, min.y+j+0.5, min.z+k+0.5);
                     if(mesh0.pointIsInside (p)==1) 
                        outputWriter.write("1"+ " ");                    
                     else                        
                        outputWriter.write("0"+ " ");
                     
                     }
                  }
               }
            outputWriter.close ();
         }
         
        }
           
      System.out.println("finished!");
      
   }
   

}