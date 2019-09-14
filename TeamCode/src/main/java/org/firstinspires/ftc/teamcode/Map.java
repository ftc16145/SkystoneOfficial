import java.util.Arrays;
// For orientation, see GM2 1.3.1
public static class Map {
    public static enum Alliance{
        blue,red;
    }
    public int[][] map;
    private int rx,ry;
    private Alliance myTeam;
    public int[] toCoord( int x, int y ){
        return new int[]{ x - 73, -y + 73 };
    }
    public static int toCoordX( int x ){
        return x - 73;
    }
    public static int toCoordY( int y ){
        return -y + 73;
    }
    public void addRobot( int rx, int ry, double a ){
        this.rx = rx;
        this.ry = ry;
        for(int x = 0; x < 146; x++){
            for(int y = 0; y < 146; y++){
                int nx = toCoord( x, y )[0];
                int ny = toCoord( x, y )[1];
                //int rotnx = nx;
                //int rotny = ny;
                double angle = (a) * (Math.PI/180); // Convert to radians
                double rotnx = Math.cos(angle) * (nx - rx) - Math.sin(angle) * (ny-ry);
                double rotny = Math.sin(angle) * (nx - rx) + Math.cos(angle) * (ny - ry);
                if(Math.abs(((rotnx-rx)/13)+((rotny-ry)/13))+Math.abs(((rotnx-rx)/13)-((rotny-ry)/13))<=2){
                    map[x][y]+=2;
                }
            }
        }
    }
    public void addForbidden( int xtl, int ytl, int xbr, int ybr ){

    }
    public void refresh(){
        for( int[] i : map ){
            Arrays.fill( i, 0 );
        }
        for( int x = 0 ; x < 146 ; x++ ) {
            for( int y = 0 ; y < 146 ; y++ ){
                int nx = toCoord( x, y )[0];
                int ny = toCoord( x, y )[1];
                int anx = Math.abs(nx);
                int any = Math.abs(ny);
                /*
                (((x-h)^2)/a^2)+((y-k)^2+/b^2)=1
                h,k = center
                a= horiz r
                b = vert r
                */
                //if( !( ( Math.pow( ( Math.abs( nx ) - 24.5 ), 2 ) / Math.pow( ( 4 / 3 ) + 13, 2 ) ) + ( Math.pow( ( ny ), 2 ) / Math.pow( 15, 2 ) ) > 1 ) ){
                //   map[x][y]=1;
                //}
                if(Math.abs(((anx-24.5)/2)+(ny/4))+Math.abs(((anx-24.5)/2)-(ny/4))<=2){
                    map[x][y]=1;
                }

                if( x == 0 || x == 145 || y == 0 || y == 145 ) map[x][y] = 1;
                /*
                 |x/p + y/q| + |x/p - y/q| = 2
                 p = horiz r
                 q = vert r
                */
                int side=0;
                switch( myTeam ){
                    case blue:
                        side = -1;
                        if( nx >= -72 && nx <= -72+25 && ny >= -72 && ny <= -72 + 25 ){
                            map[x][y] = 1;
                        }
                        break;
                    case red:
                        side = 1;
                        if( nx <= 72 && nx >= 72-25 && ny >= -72 && ny <= -72 + 25 ){
                            map[x][y] = 1;
                        }
                        break;
                }
                if(  Math.abs( ( ( nx+(36*side) ) / 18 ) + ( ( ny - 36) / 18 ) ) + Math.abs( ( ( nx+(36*side) ) / 18 ) - ( ( ny - 36 ) / 18 ) ) <= 2){
                    map[x][y] = 1;
                }


                if( ny >= 8 && ny <= 40){
                    switch( myTeam ){
                        case blue:
                            if( 0 <= nx && nx <= 22.5 ){
                                map[x][y] = 0;
                            }
                            break;
                        case red:
                            if( -22.5 <= nx && nx <= 0 ){
                                map[x][y] = 0;
                            }
                            break;
                    }
                }
                if(Math.abs(((anx)/24.5)+(ny/4))+Math.abs(((anx)/24.5)-(ny/4))<=2){
                    map[x][y]=0;
                }

            }
        }
    }
    public Map( Alliance a ){
        myTeam = a;
        map = new int[146][146];
        refresh();
    }
    public boolean isConflict(){
        boolean found = false;
        for(int x = 0; x < 146; x++){
            for(int y = 0; y < 146; y++){
                if(map[x][y]==3){
                    found = true;
                }
            }
        }
        return found;
    }

}
