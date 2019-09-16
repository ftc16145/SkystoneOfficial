public static class Path{
   int[][] points;
   int stage;
   int[] stageP;
   boolean pathDone;
   public Path(int[]... points){
     this.points = points;
     stage=0;
     stageP = points[stage];
     pathDone = false;
   }
   public int getNumberPts(){
     return points.length;
   }
   public void addPoint(int[] pt){
     points[points.length+1]=pt;
   }
   public double[] getXY( double currentX, double currentY ){
     double m = (stageP[1]-currentY)/(stageP[0]-currentX);
       if(m==0 ){
         return new double[]{Math.signum(stageP[0]-currentX),0};
       }else if(m>1){
         return new double[]{1/m,1};
       }else if(Double.isNaN(m)){
         return new double[]{0,Math.signum(stageP[1]-currentY)};
       }else{
         return new double[]{1,m};
       }   }
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
}
