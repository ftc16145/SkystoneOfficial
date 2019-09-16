public static class X2YPathFinder extends PathFinder{
  
  public Path findPath( int[] start, int[] end, Map.Alliance a ){
      super.pathSetup(start,end,a);
      if(!goodPath){
        return new Path(start);
      } else{
        p = new Path(new int[]{end[0],start[1]},end);
        for(int x = (int) rx; x < p.getStageP()[0];x++){
          if(goodPath){
            m.refresh();
            m.addRobot(rx,ry);
            if(m.isConflict()){
              System.out.println("X2Y Failed.");
              goodPath = false;
          
            }else{
              rx+=p.getXY(rx,ry)[0];
              System.out.println("X2Y Still Good");
            }
          }
        }
        for(int y = (int) rx; y < p.getStageP()[1];y++){
          if(goodPath){
            m.refresh();
            m.addRobot(rx,ry);
            if(m.isConflict()){
              System.out.println("X2Y Failed.");
              goodPath = false;
          
            }else{
              ry+=p.getXY(rx,ry)[1];
              System.out.println("X2Y Still Good");
            }
          }
        }
        if(goodPath){
          System.out.println("X2Y Returned");
          return p;
        }else{
          return new Path(start);
        }
    
      }
    }
  }
