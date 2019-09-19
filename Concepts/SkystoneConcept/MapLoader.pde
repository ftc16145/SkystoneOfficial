import java.io.FileReader;

class MapLoader {
    
    public int[][] load( String filename ) throws IOException {
        int[][] map = new int[144][144];
        
        BufferedReader br = new BufferedReader(new FileReader( filename ) );
        String line = br.readLine();
        for( int j=0; line != null && j<144; j++ ) {
            for( int i=0; i<line.length() && i<144; ++i ) {
                map[j][i] = line.charAt(i) - '0';
            }
            
            line = br.readLine();
        }
        br.close();
        
        return map;
    }
}
