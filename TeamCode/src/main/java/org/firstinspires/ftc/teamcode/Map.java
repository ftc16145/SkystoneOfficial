package org.firstinspires.ftc.teamcode;
import java.util.Arrays;

public class Map {
    private int[][] map;
    private int[] toCoord( int x, int y ){
        return new int[]{ x-72, -y+72 };
    }
    public void addRobot( int x, int y ){

    }
    public void addForbidden( int xtl, int ytl, int xbr, int ybr ){
        
    }
    public Map(){
        map = new int[144][144];
        for(int[]a : map){
            Arrays.fill(a, 0);
        }
        for(int x = 0 ; x < 144; x++) {
            for(int y = 0; y < 144; y++){
                int nx = toCoord(x,y)[0];
                int ny = toCoord(x,y)[1];
                if(!((Math.pow((Math.abs(nx)-48),2)/Math.pow(15,2))+(Math.pow((ny),2)/Math.pow((4/3)+13,2))>1)){
                    map[x][y]=1;
                }
            }
        }
    }
}
