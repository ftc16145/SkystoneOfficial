public static class PathFinder{
  protected static Path p;
  protected static boolean found = false;
  protected static int[] s, e;
  protected static Map m;
  protected static boolean goodPath = true;
  protected static double rx, ry;
  
  
  
  public Path findPath( int[] start, int[] end, Map.Alliance a ){
    pathSetup(start,end,a);
    return new Path(start);
    
  }
  private void pathSetup(int[] start, int[] end, Map.Alliance a){
    m = new Map(a);
    m.refresh();
    rx=start[0];
    ry=start[1];
    m.addRobot(start);
    if( m.map[Map.antiCoordX(end[0])][Map.antiCoordY(end[1])]==1){
      goodPath = false;
    }
  }
}
