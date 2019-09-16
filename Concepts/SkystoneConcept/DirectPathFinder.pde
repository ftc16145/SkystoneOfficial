public static class DirectPathFinder extends PathFinder{
  //private static Path p;
  //private static boolean found = false;
  //private static int[] s, e;
  //private static Map m;
  @Override
  public Path findPath(int[] start, int[] end, Map.Alliance a){
    super.pathSetup(start,end,a);
    if(!goodPath){
        return new Path(start);
    } else{
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
          goodPath = false;
        }else{
          rx+=direct[0];
          ry+=direct[1];
        }
      }
      if(goodPath){
        return p;
      } else {
        return new Path(start);
      }

    }
  }
}
