Map m;
void setup(){
  size(146,146);
  m = new Map(Map.Alliance.red);
}
void draw(){
  noStroke();
  background(255,0,255);
  m.refresh();
  m.addRobot(Map.toCoordX(mouseX),Map.toCoordY(mouseY));
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
  System.out.println(m.isConflict());
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
