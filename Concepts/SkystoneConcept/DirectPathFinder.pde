public static class DirectPathFinder extends PathFinder{
  //private static Path p;
  //private static boolean found = false;
  //private static int[] s, e;
  //private static Map m;
  @Override
  public Path findPath(int[] start, int[] end, Map.Alliance a){
    pathSetup(start,end,a);
    if(!goodPath){
      System.out.println("I can't do that!");
        return new Path(s);
    } else{
      p = new Path(e);
      System.out.println("Staring testStage");
      super.testStage(p);
      if(goodPath){
        System.out.println("Direct is Valid.");
        return p;
      } else {
        System.out.println("Direct is NOT Valid.");
        throw new IllegalArgumentException("Invalid Path");
      }

    }
  }
}
