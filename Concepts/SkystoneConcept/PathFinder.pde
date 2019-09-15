public static class PathFinder{
  private static Path p;
  private static boolean found = false;
  private static int[] s, e;
  private static Map m;
  private static Path findPath(int[] start, int[] end, Map.Alliance a){
    boolean directGood = true;
    boolean xToYGood = true;
    double rx=start[0],ry=start[1];
    m = new Map(a);
    m.addRobot(start);
    p = new Path(end);
    double[] direct = p.getXY(start);
    if( m.map[Map.antiCoordX(end[0])][Map.antiCoordY(end[1])]==1){
      return new Path(start);
    }
    for(int x = (int) rx; x < end[0]; x++){
      m.refresh();
      m.addRobot(rx,ry);
      if(m.isConflict()){
        System.out.println("Direct Failed.");
        directGood = false;
        
      }else{
        rx+=direct[0];
        ry+=direct[1];
      }
    }
    if(directGood){
      return p;
    } else {
      rx=start[0];
      ry=start[1];
      m.refresh();
      m.addRobot(rx,ry);
     
      p = new Path(new int[]{end[0],start[1]},end);
      for(int x = (int) rx; x < p.getStageP()[0];x++){
        m.refresh();
        m.addRobot(rx,ry);
        if(m.isConflict()){
          System.out.println("X2Y Failed.");
          xToYGood = false;
          
        }else{
          rx+=p.getXY(rx,ry)[0];
          System.out.println("X2Y Still Good");
        }
      }
      for(int y = (int) rx; y < p.getStageP()[1];y++){
        m.refresh();
        m.addRobot(rx,ry);
        if(m.isConflict()){
          System.out.println("X2Y Failed.");
          xToYGood = false;
          
        }else{
          ry+=p.getXY(rx,ry)[1];
          System.out.println("X2Y Still Good");
        }
      }
      if(xToYGood){
        System.out.println("X2Y Returned");
        return p;
      }else{
        return new Path(start);
      }
    }

}
  
}
