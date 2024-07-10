/**
 * Copyright (c) 2020, by the Author: Rui Yang (UBC)
 */
package artisynth.istar.Rui;

import maspack.geometry.Vertex3d;


class GeoVertex {
   Vertex3d myVert;
   double myCost;
  
   GeoVertex preVert = null;
   
   public GeoVertex(Vertex3d vert,  double cost) {
      super();
      this.myVert = vert;
      this.myCost = cost;
    }
   
   public Vertex3d getVert() {
      return myVert;
    }

    public double getCost() {
      return myCost;
    }
    
    public void setVert(Vertex3d vert) {
       myVert = vert;
    }
    
    public void setCost(double cost) {
       myCost = cost;
    }
    
    public void setPreVert(GeoVertex pVert) {
       preVert = pVert;
    }

    public GeoVertex getPreVert() {
       return preVert;
    }
}