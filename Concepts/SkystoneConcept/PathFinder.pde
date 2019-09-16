public static class PathFinder{
  protected static Path p;
  protected static boolean found = false;
  protected static int[] s, e;
  protected static Map m;
  protected static boolean goodPath = true;
  protected static double rx, ry;
  
  public PathFinder(){}
  
  public Path findPath( int[] start, int[] end, Map.Alliance a ){
    pathSetup(start,end,a);
    return new Path(start);
    
  }
  
  protected void pathSetup(int[] start, int[] end, Map.Alliance a){
    s = start;
    e = end;
    goodPath = true;
    m = new Map(a);
    m.refresh();
    m.addRobot(s);
    rx=start[0];
    ry=start[1];
    if( m.map[Map.antiCoordX(e[0])][Map.antiCoordY(e[1])]==1){
      goodPath = false;
    }
    
  }
  protected void testStage(Path p){
    this.p=p;
    System.out.println("Running testStage");
    for(int s =0; s < p.getNumberPts();s++){
      boolean running = true;
      double[] pxy = p.getXY(rx,ry);
      for(int i = 0;i < pxy.length;i++){
        if(pxy[i] == Double.NaN){
          pxy[i]=0;
        }
      }
      
      System.out.println("pxy = " + pxy[0] + ", " +pxy[1]);
      while(running){
        if( pxy[0] == 0 && pxy[1] ==0 ){
          running = false;
          break;
        }
          //System.out.println("Running tester");
          m.refresh();
          m.addRobot(rx,ry);
          if(m.isConflict()){
            System.out.println("Method Failed.");
            goodPath = false;
            break;
          }else{
            System.out.println("Good to move again, at " + rx + ", " + ry);
            rx+=pxy[0];
            ry+=pxy[1];
          }
      
     
        System.out.println("Starting new stage!");
        p.nextStage();
      }
    }  
    
  }
}
