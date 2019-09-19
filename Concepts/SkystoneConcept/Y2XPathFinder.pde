public static class Y2XPathFinder extends PathFinder{
  
  public Path findPath( int[] start, int[] end, Map.Alliance a ){
      pathSetup(start,end,a);
      if(!goodPath){
        System.out.println("I can't do that!");
        return new Path(s);
      } else{
        p = new Path(new int[]{s[0],e[1]},e);
        super.testStage(p);
        if(goodPath){
          System.out.println("Y2X Returned");
          return p;
        }else{
          throw new IllegalArgumentException("Invalid Path");
        }
    
      }
    }
  }
