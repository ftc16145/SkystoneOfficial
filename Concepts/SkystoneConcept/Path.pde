public static class Path{
   int[][] points;
   int stage;
   int[] stageP;
   boolean pathDone;
   boolean stageDone;
   public Path(int[]... points){
     this.points = points;
     stage=0;
     stageP = points[stage];
     pathDone = false;
   }
   public int getNumberPts(){
     return points.length;
   }
   public int[][] getAllPts(){
     return points;
   }
   public void addPoint(int[] pt){
     points[points.length+1]=pt;
   }
   public double[] getXY( double currentX, double currentY ){
     double m = (stageP[1]-currentY)/(stageP[0]-currentX);
     //System.out.println(m);
       if(m==0 ){
         System.out.println("Horizontal");
         return new double[]{Math.signum(stageP[0]-currentX),0};
       }else if(Math.abs(m)>1){
         return new double[]{1/m,Math.signum(stageP[1]-currentY)};
       }else if(Double.isNaN(m)){
         System.out.println("Verical, I need to get to "+ stageP[0] + ", " + stageP[1] + ", but I'm at " + currentX + ", " + currentY );
         if(stageP[1]-currentY<0){
           return new double[]{0,-1};
         }else if (stageP[1]-currentY>0){
           return new double[]{0,1};
         }else if(Math.abs(m) < 1){
           return new double[]{Math.signum(stageP[0]-currentX),m};
       }else{
           stageDone = true;
           return new double[]{0,0};
         }
       }else{
         return new double[]{1,m};
       }   
     }
   public double[] getXY( int[] currentP ){
     return getXY(currentP[0],currentP[1]);
   }
   public void nextStage(){
    
     if(stage==points.length-1){
       pathDone = true;
     }else{
       stage++;
       stageP=points[stage];
     }
     stageDone = false;
   }
   public int[] getStageP(){
     return stageP;
   }
   public int getStage(){
     return stage;
   }
   public boolean isDone(){
     return pathDone;
   }
   public boolean getStageDone(){
     return stageDone;
   }
}
