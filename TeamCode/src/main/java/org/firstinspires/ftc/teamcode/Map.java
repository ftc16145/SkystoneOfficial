package org.firstinspires.ftc.teamcode;
import java.util.Arrays;
// For orientation, see GM2 1.3.1
public class Map {
    public enum Alliance{
        blue,red;
    }
    private int[][] map;
    private Alliance myTeam;
    private int[] toCoord( int x, int y ){
        return new int[]{ x - 72, -y + 72 };
    }
    public void addRobot( int rx, int ry ){
        
    }
    public void addForbidden( int xtl, int ytl, int xbr, int ybr ){
        
    }
    public Map( Alliance a ){
        myTeam = a;
        map = new int[146][146];
        for( int[] a : map ){
            Arrays.fill( a, 0 );
        }
        for( int x = 0 ; x < 146 ; x++ ) {
            for( int y = 0 ; y < 146 ; y++ ){
                int nx = toCoord( x, y )[0];
                int ny = toCoord( x, y )[1];
                /*
                (((x-h)^2)/a^2)+((y-k)^2+/b^2)=1
                h,k = center
                a= horiz r
                b = vert r
                */
                if( !( ( Math.pow( ( Math.abs( nx ) - 48 ), 2 ) / Math.pow( 15, 2 ) ) + ( Math.pow( ( ny ), 2 ) / Math.pow( ( 4 / 3 ) + 13, 2 ) ) > 1 ) ){
                    map[x][y]=1;
                }
                if( x == 0 || x == 145 || y == 0 || y == 145 ) map[x][y] = 1;
                /*
                 |x/p + y/q| + |x/p - y/q| = 2
                 p = horiz r
                 q = vert r
                */
                double int side;
                switch( a ){
                    case blue:
                        side = -1;
                        break;
                    case red:
                        side = 1;
                        break;
                 }
                 if( !( Math.abs( ( ( nx+(36*side) ) / 36 ) + ( ( ny - 48) / 24 ) ) + Math.abs( ( ( nx+(36*side) ) / 36 ) - ( ( ny - 48 ) / 24 ) ) ) > 2){
                    map[x][y] = 1;
                 }
          
            }
        }
    }
}
