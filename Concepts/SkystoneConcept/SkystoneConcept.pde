Map m;
int[] roboP;
int[][] test;
Path currentPath;
boolean havePath;
PathFinder[] paths;
void setup(){
  size(146,146);
  m = new Map(Map.Alliance.blue);
  roboP = m.toCoord(14,72-24);
  
  paths = new PathFinder[]{new DirectPathFinder(), new X2YPathFinder()};
}
void draw(){
  noStroke();
  background(255,0,255);
  if(havePath){
    if(currentPath.isDone()){
      System.out.println("Done!");
    }else{
        System.out.println("Current Running Path on Stage: " + currentPath.getStage() + ", XY = " + currentPath.getXY(roboP)[0]+", "+currentPath.getXY(roboP)[1]);
      
      roboP[0]+=currentPath.getXY(roboP)[0];
      roboP[1]+=currentPath.getXY(roboP)[1];
    }
  }
  //System.out.println(havePath);
  m.refresh();
  
  m.addRobot(roboP);
  
  for(int x = 0; x < 146; x++){
    for(int y = 0; y < 146; y++){
      switch(m.map[x][y]){
        case(0):
          fill(255);
          break;
        case(1):
          fill(0);
          break;
        case(2):
          fill(0,255,0);
          break;
        case(3):
          fill(255,0,0);
          break;
      }
      rect(x,y,1,1);       
    }
  }
  //System.out.println(m.isConflict());
  noStroke();
  fill(168,155,50);
  //rect(1+46,(height/2)-8,45+8,16);
  stroke(0,255,0);
  line(width/2,1,width/2,height-1);
  stroke(0,0,255);
  line(1,height/2,47,height/2);
  line(0,23,23,0);
  noFill();
  rect(width-25,height-25,24,24);
  stroke(255,255,0);
  line(51,height/2,96,height/2);
  stroke(255,0,0);
  line(100,height/2,width-1,height/2);
  line(width-23,0,width-1,23);
  rect(1,height-25,24,24);
  line(width/2,height/2,width/2,(height/2)+16);
  line(mouseX,mouseY,mouseX-13,mouseY);
} 
void mousePressed(){
  Path errorPath = new Path(new int[]{(int) Double.NaN, (int) Double.NaN});
  havePath = false;
  for(int i = 0; i < paths.length; i++){
    System.out.println("Trying path " + i);
     //paths[i].findPath(roboP, new int[]{Map.toCoordX(mouseX),Map.toCoordY(mouseY)},Map.Alliance.blue)
     try{
       currentPath = paths[i].findPath(roboP, new int[]{0,-36},Map.Alliance.blue);
       havePath = true;
       break;
     }
     catch(IllegalArgumentException ex){
       System.out.println("Error on " + i + ": " + ex.getMessage());
     }
  }
  System.out.println(havePath);
  //currentPath = PathFinder.findPath(roboP,new int[]{0,-36},Map.Alliance.blue);
  //if(currentPath==new Path(roboP)){
  //  System.out.println("No Path Found");
  //  havePath=false;
  //} else havePath=true;
}
