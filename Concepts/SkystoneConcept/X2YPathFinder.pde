public static class X2YPathFinder extends PathFinder{
  
  public Path findPath( int[] start, int[] end, Map.Alliance a ){
      pathSetup(start,end,a);
      if(!goodPath){
        return new Path(s);
      } else{
        p = new Path(new int[]{e[0],s[1]});
        super.testStage(p);
        if(goodPath){
          System.out.println("X2Y Returned");
          return p;
        }else{
          throw new IllegalArgumentException("Invalid Path");
        }
    
      }
    }
  }
